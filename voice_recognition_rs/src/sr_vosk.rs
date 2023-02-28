use std::thread::spawn;

use crossbeam_channel::{Receiver, Sender};
use vosk::{Model, Recognizer};
use vosk::DecodingState::*;

use crate::config::VoskConfig;
use crate::speech_recognition::{ResultSource, SpeechRecognizer};
use crate::speech_recognition::ResultSource::Vosk;

pub struct VoskRecognizer {
    audio_receiver: Receiver<Vec<i16>>,
    partial_results: Sender<(ResultSource, String)>,
    final_results: Sender<(ResultSource, String)>,
    recognizer: Option<Recognizer>,
    model_path: String,
    sample_rate: u32,
}

impl VoskRecognizer {
    pub(crate) fn new(audio_receiver: Receiver<Vec<i16>>, partial_results: Sender<(ResultSource, String)>, final_results: Sender<(ResultSource, String)>, config: VoskConfig) -> VoskRecognizer {
        VoskRecognizer {
            audio_receiver,
            partial_results,
            final_results,
            recognizer: None,
            model_path: config.model_path,
            sample_rate: config.sample_rate,
        }
    }
}

impl SpeechRecognizer for VoskRecognizer {
    fn load(&mut self) {
        println!("[Vosk] Loading Vosk Model {}", self.model_path);

        let model = Model::new(self.model_path.to_owned()).expect("Could not create the model");

        let mut recognizer = Recognizer::new(&model, self.sample_rate as f32)
            .expect("Could not create the Recognizer");

        recognizer.set_max_alternatives(0);
        recognizer.set_words(false);
        recognizer.set_partial_words(false);

        self.recognizer = Some(recognizer);

        println!("[Vosk] Loading done!");
    }

    fn start(self) {
        spawn(move || {
            let mut recognizer = self.recognizer.unwrap();
            let mut last_result = String::new();
            for data in self.audio_receiver {
                let state = recognizer.accept_waveform(&data);
                match state {
                    Running => {
                        let partial = recognizer.partial_result();
                        if !partial.partial_result.is_empty() || !partial.partial.is_empty() {
                            if partial.partial == last_result || partial.partial.chars().nth(0) == last_result.chars().nth(0) {
                                continue;
                            }
                            last_result = partial.partial.to_owned();
                            self.partial_results.send((Vosk, partial.partial.to_owned())).unwrap();
                        }
                    }
                    Finalized => {
                        // Result will always be multiple because we called set_max_alternatives
                        if let Some(result) = recognizer.result().single() {
                            let result = result.text.trim().to_owned();
                            if !result.is_empty() {
                                if result.len() == 1 {
                                    if result != last_result && result.chars().nth(0) != last_result.chars().nth(0) {
                                        last_result = result.clone();
                                        self.partial_results.send((Vosk, result.clone())).unwrap();
                                    }
                                } else {
                                    self.final_results.send((Vosk, result)).unwrap();
                                }
                            }
                        }
                    }
                    Failed => eprintln!("error"),
                }
            }
        });
    }
}
