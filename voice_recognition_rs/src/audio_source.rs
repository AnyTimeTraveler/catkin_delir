use std::io::Read;
use std::net::TcpStream;
use libpulse_binding::def::BufferAttr;
use libpulse_binding::sample::Spec;
use libpulse_binding::stream::Direction;
use libpulse_simple_binding::Simple;

#[cfg(feature = "localplayback")]
use std::fs::File;

#[cfg(feature = "localplayback")]
use minimp3::{Error, Frame};

pub(crate) fn convert_u8_to_i16(data_in: &[u8], samples_out: &mut [i16]) {
    assert_eq!(data_in.len() / 2, samples_out.len());
    data_in.chunks(2)
        .map(|a| i16::from_le_bytes(a.try_into().unwrap()))
        .enumerate()
        .for_each(|(i, sample)| samples_out[i] = sample);
}

struct PulseAudioSource {
    buffer: Vec<u8>,
    source: Simple,
}

struct TCPAudioSource {
    buffer: Vec<u8>,
    source: TcpStream,
}

pub(crate) trait AudioSource {
    fn read(&mut self, sample_buffer: &mut [i16]);
}

impl AudioSource for PulseAudioSource {
    fn read(&mut self, sample_buffer: &mut [i16]) {
        self.source.read(&mut self.buffer).unwrap();
        convert_u8_to_i16(&self.buffer, sample_buffer);
    }
}

impl AudioSource for TCPAudioSource {
    fn read(&mut self, sample_buffer: &mut [i16]) {
        self.source.read(&mut self.buffer).unwrap();
        convert_u8_to_i16(&self.buffer, sample_buffer);
    }
}

#[cfg(feature = "localplayback")]
impl AudioSource for minimp3::Decoder<File> {
    fn read(&mut self, sample_buffer: &mut [i16]) {
        match self.next_frame() {
            Ok(Frame { data, channels, .. }) => {
                assert_eq!(data.len() / channels, sample_buffer.len());
                for i in 0..sample_buffer.len() {
                    sample_buffer[i] = data[i * channels];
                }
            }
            Err(Error::Eof) => {}
            Err(e) => panic!("{:?}", e),
        }
    }
}

pub(crate) fn open_direct_microphone(spec: &Spec, buffer_size: usize) -> Box<dyn AudioSource> {
    Box::new(PulseAudioSource {
        buffer: vec![0u8; buffer_size as usize * 2],
        source: Simple::new(
            None,                       // Use the default server
            "Delirium Recognition",     // Our application’s name
            Direction::Record,          // We want a playback stream
            None,                       // Use the default device
            "Delirium Recognition",             // Description of our stream
            spec,                      // Our sample format
            None,                       // Use default channel map
            Some(
                &BufferAttr {
                    maxlength: (buffer_size * 2) as u32,
                    tlength: 0,
                    prebuf: 0,
                    minreq: 0,
                    fragsize: buffer_size as u32,
                }
            ),                       // Use default buffering attributes
        ).unwrap(),
    })
}

pub(crate) fn open_tcp_microphone(buffer_size: usize, address: String) -> Box<dyn AudioSource> {
    Box::new(TCPAudioSource {
        buffer: vec![0u8; buffer_size * 2],
        source: TcpStream::connect(address)
            .expect("Error creating TCP stream for TCP microphone!"),
    })
}

#[cfg(feature = "localplayback")]
pub(crate) fn open_audiofile(file: String) -> Box<dyn AudioSource> {
    Box::new(minimp3::Decoder::new(File::open(file)
        .expect("Error opening file for FILE audiosource!")))
}
