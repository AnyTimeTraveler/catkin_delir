use enum_display_derive::Display;
use std::fmt::Display;

pub trait SpeechRecognizer {
    fn load(self: &mut Self);
    fn start(self: Self);
}

#[allow(unused)]
#[derive(Display)]
pub enum ResultSource {
    Vosk,
    DeepSpeech,
    PocketSphinx
}
