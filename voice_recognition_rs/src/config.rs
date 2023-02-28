use std::fs::File;
use std::io::Read;
use serde::Deserialize;

#[allow(unused)]
#[derive(Deserialize, Debug)]
pub(crate) struct Config {
    pub(crate) audio_source: AudioSource,
    pub(crate) output: Output,
    pub(crate) vosk: VoskConfig,
    pub(crate) pocketsphinx: PocketsphinxConfig,
}

#[allow(unused)]
#[derive(Deserialize, Debug)]
pub(crate) struct AudioSource {
    pub(crate) mode: Mode,
    pub (crate) file: Option<String>,
    pub(crate) address: Option<String>,
    pub(crate) buffer_size: usize,
    pub(crate) input_sample_rate: u32,
    pub(crate) listen_to_input: bool,
    pub(crate) filter_noise: bool,
}

#[derive(Deserialize, Debug)]
pub(crate) struct Output {
    pub(crate) listen_address: String,
}

#[derive(Deserialize, Debug)]
pub(crate) enum Mode {
    DIRECT,
    FILE,
    TCP,
}

#[allow(unused)]
#[derive(Deserialize, Debug)]
pub(crate) struct VoskConfig {
    pub(crate) enabled: bool,
    pub(crate) sample_rate: u32,
    pub(crate) model_path: String,
}

#[allow(unused)]
#[derive(Deserialize, Debug)]
pub(crate) struct PocketsphinxConfig {
    pub(crate) enabled: bool,
    pub(crate) sample_rate: u32,
    pub(crate) hmm_path: String,
    pub(crate) lm_path: String,
    pub(crate) dict_path: String,
}

pub(crate) fn read_config() -> Config {
    let mut config = String::new();
    File::open("config.toml")
        .expect("No config file found!")
        .read_to_string(&mut config)
        .expect("Error reading config file!");

    toml::from_str(&config).expect("Error parsing config!")
}
