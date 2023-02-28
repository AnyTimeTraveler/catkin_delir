use std::io::Write;
use std::net::{TcpListener, TcpStream};
use crossbeam_channel::{bounded, Receiver, Sender, unbounded};
use std::thread::spawn;
use libpulse_binding::def::BufferAttr;
use libpulse_binding::sample::{Format, Spec};
use libpulse_binding::stream::Direction;
use libpulse_simple_binding::Simple;
use crate::audio_conversion::AudioConverter;
use crate::audio_source::{open_direct_microphone, open_tcp_microphone};
use crate::config::{Config, Mode, read_config};
use crate::speech_recognition::{ResultSource, SpeechRecognizer};
#[cfg(feature = "localplayback")]
use crate::audio_source::open_audiofile;

#[cfg(feature = "noisefilter")]
use nnnoiseless::DenoiseState;
#[cfg(feature = "noisefilter")]
use crate::noise_suppression::Filter;
#[cfg(feature = "vosk")]
use crate::sr_vosk::VoskRecognizer;
#[cfg(feature = "pocketsphinx")]
use crate::sr_pocketsphinx::PocketSphinxRecognizer;

#[cfg(feature = "noisefilter")]
mod noise_suppression;
#[cfg(feature = "vosk")]
mod sr_vosk;
#[cfg(feature = "pocketsphinx")]
mod sr_pocketsphinx;

mod speech_recognition;
mod config;
mod audio_source;

pub(crate) mod audio_conversion;

fn main() {
    let config: Config = read_config();
    let audio_source_config = config.audio_source;
    let spec = create_valid_spec(audio_source_config.input_sample_rate);

    let mut input_stream = match audio_source_config.mode {
        Mode::DIRECT => open_direct_microphone(&spec, audio_source_config.buffer_size),
        Mode::TCP => open_tcp_microphone(audio_source_config.buffer_size, audio_source_config.address.clone().expect("Expected address for TCP microphone in config!")),
        Mode::FILE => {
            #[cfg(not(feature = "localplayback"))]
            panic!("Not compiled with feature localplayback!");
            #[cfg(feature = "localplayback")]
            open_audiofile(audio_source_config.file.expect("Expected file for audio source!"))
        }
    };

    let sink = if audio_source_config.listen_to_input {
        Some(create_sink(&spec, audio_source_config.buffer_size as u32))
    } else {
        None
    };

    let (partial_result_sender, partial_result_receiver) = unbounded();
    let (final_result_sender, final_result_receiver) = unbounded();

    let mut audio_senders: Vec<AudioConverter> = Vec::new();

    #[cfg(feature = "vosk")]
    if config.vosk.enabled {
        let (audio_sender, audio_receiver) = unbounded();
        audio_senders.push(AudioConverter {
            input_sample_rate: audio_source_config.input_sample_rate,
            output_sample_rate: config.vosk.sample_rate,
            sender: audio_sender,
        });
        let mut recognizer = VoskRecognizer::new(audio_receiver, partial_result_sender.clone(), final_result_sender.clone(), config.vosk);

        spawn(move || {
            recognizer.load();
            recognizer.start();
        });
    }

    #[cfg(feature = "pocketsphinx")]
    if config.pocketsphinx.enabled {
        let (audio_sender, audio_receiver) = unbounded();
        audio_senders.push(AudioConverter {
            input_sample_rate: audio_source_config.input_sample_rate,
            output_sample_rate: config.pocketsphinx.sample_rate,
            sender: audio_sender,
        });

        let mut recognizer = PocketSphinxRecognizer::new(audio_receiver, partial_result_sender.clone(), final_result_sender.clone(), config.pocketsphinx);
        spawn(move || {
            recognizer.load();
            recognizer.start();
        });
    }

    create_result_sender(partial_result_receiver, final_result_receiver, config.output.listen_address.clone());

    let mut samples = vec![0i16; audio_source_config.buffer_size];
    let mut out_samples = vec![0i16; audio_source_config.buffer_size];

    #[cfg(feature = "noisefilter")]
    let mut denoise = DenoiseState::new();

    loop {
        input_stream.read(&mut samples);

        if audio_source_config.filter_noise {
            #[cfg(feature = "noisefilter")]
            denoise.filter_audio(&samples, &mut out_samples);
            #[cfg(not(feature = "noisefilter"))]
            panic!("Not compiled with feature noisefilter!");
        } else {
            for i in 0..samples.len() {
                out_samples[i] = samples[i];
            }
        }

        if let Some(sink) = &sink {
            let bytes: Vec<u8> = out_samples.iter()
                .flat_map(|data| data.to_le_bytes())
                .collect();
            sink.write(&bytes).unwrap();
        }

        for audio_sender in &audio_senders {
            audio_sender.send(&out_samples).unwrap();
        }
    }
}

fn create_result_sender(partial_results: Receiver<(ResultSource, String)>, final_results: Receiver<(ResultSource, String)>, listen_address: String) {
    let (tcp_sender, tcp_receiver) = bounded(1024);
    let tcp_sender2 = tcp_sender.clone();
    spawn(move || {
        for result in partial_results {
            send_message(result, 'T', &tcp_sender);
        }
    });
    spawn(move || {
        for result in final_results {
            send_message(result, 'F', &tcp_sender2);
        }
    });
    spawn(move || {
        let net = TcpListener::bind(listen_address).expect("Expected valid listen_address in output config!");
        loop {
            for connection in net.incoming() {
                if let Ok(mut connection) = connection {
                    // empty the connection once before receiving new recognitions
                    while let Ok(_) = tcp_receiver.try_recv() {}

                    if let Err(error) = forward_data(&tcp_receiver, &mut connection) {
                        eprintln!("{error}");
                        continue;
                    }
                }
            }
        }
    });
}

fn forward_data(receiver: &Receiver<String>, sender: &mut TcpStream) -> Result<(), String> {
    loop {
        let string = receiver.recv().map_err(|e| format!("A{:?}", e))?;
        sender.write_all(string.as_bytes()).map_err(|e| format!("B{:?}", e))?;
        sender.write_all("\n".as_bytes()).map_err(|e| format!("C{:?}", e))?;
        sender.flush().map_err(|e| format!("D{:?}", e))?;
    }
}

fn send_message(msg: (ResultSource, String), result_type: char, target: &Sender<String>) {
    let string = format!("{}:{}:{}", result_type, msg.0, msg.1);
    println!("Sending: {}", string);
    target.try_send(string).unwrap();
}

fn create_sink(spec: &Spec, buffer_size: u32) -> Simple {
    Simple::new(
        None,                        // Use the default server
        "Delirium Recognition",      // Our application’s name
        Direction::Playback,         // We want a playback stream
        None,                        // Use the default device
        "Audio Playback",            // Description of our stream
        &spec,                       // Our sample format
        None,                        // Use default channel map
        Some(
            &BufferAttr {
                maxlength: buffer_size * 2,
                tlength: 0,
                prebuf: buffer_size,
                minreq: buffer_size,
                fragsize: buffer_size,
            }
        ),                       // Use default buffering attributes
    ).unwrap()
}

fn create_valid_spec(sample_rate: u32) -> Spec {
    let spec = Spec {
        format: Format::S16le,
        channels: 1,
        rate: sample_rate,
    };
    assert!(spec.is_valid());
    spec
}
