use std::thread::spawn;
use crossbeam_channel::{Receiver, Sender};
use pocketsphinx::{CmdLn, PsDecoder};
use crate::config::PocketsphinxConfig;
use crate::speech_recognition::{ResultSource, SpeechRecognizer};
use crate::speech_recognition::ResultSource::PocketSphinx;

#[allow(unused)]
pub struct PocketSphinxRecognizer {
    audio_receiver: Receiver<Vec<i16>>,
    partial_results: Sender<(ResultSource, String)>,
    final_results: Sender<(ResultSource, String)>,
    hmm_path: String,
    lm_path: String,
    dict_path: String,
    sample_rate: u32,
}

impl PocketSphinxRecognizer {
    pub(crate) fn new(audio_receiver: Receiver<Vec<i16>>, partial_results: Sender<(ResultSource, String)>, final_results: Sender<(ResultSource, String)>, config: PocketsphinxConfig) -> PocketSphinxRecognizer {
        PocketSphinxRecognizer {
            audio_receiver,
            partial_results,
            final_results,
            hmm_path: config.hmm_path,
            lm_path: config.lm_path,
            dict_path: config.dict_path,
            sample_rate: config.sample_rate,
        }
    }
}

impl SpeechRecognizer for PocketSphinxRecognizer {
    fn load(&mut self) {
    }

    fn start(self) {
        spawn(move || {
            println!("[Pock] Loading PocketSphinx Model {}", self.lm_path);
            // let sample_rate_str =
            let ps_config =
                CmdLn::init(true, &["pocketsphinx",
                    "-hmm", &self.hmm_path,
                    "-lm", &self.lm_path,
                    "-dict", &self.dict_path,
                    // "-samprate", &sample_rate_str,
                    // "-"
                ]).expect("Error loading model!");

            let recognizer = PsDecoder::init(ps_config);
            recognizer.start_utt(Some("utt_id")).unwrap();
            println!("[Pock] Loading done!");
            let mut last_result = String::new();

            for data in self.audio_receiver {
                recognizer.process_raw(&data, false, false).unwrap();
                match recognizer.get_hyp() {
                    None => {},
                    Some((result, _utt_id, _score)) => {
                        if last_result != result {
                            last_result = result.clone();
                            self.partial_results.send((PocketSphinx, result)).unwrap();
                        }
                    }
                }
            }
        });
    }
}
