mkdir -p models
cd models || exit

function model() {
    wget "$1"
    if ! test -z "$2"; then
      mv "download" "$2"
    fi
    unzip ./*.zip
    rm ./*.zip
    tar xf ./*.tar.gz
    rm ./*.tar.gz
    gunzip ./*.gz
}

# List models below

model https://alphacephei.com/vosk/models/vosk-model-de-0.21.zip
model https://alphacephei.com/vosk/models/vosk-model-de-tuda-0.6-900k.zip
model https://alphacephei.com/vosk/models/vosk-model-small-de-zamia-0.3.zip
model https://alphacephei.com/vosk/models/vosk-model-small-de-0.15.zip
#model https://sourceforge.net/projects/cmusphinx/files/Acoustic%20and%20Language%20Models/German/cmusphinx-de-voxforge-5.2.tar.gz/download cmusphinx-de-voxforge-5.2.tar.gz
#model https://sourceforge.net/projects/cmusphinx/files/Acoustic%20and%20Language%20Models/German/cmusphinx-de-ptm-voxforge-5.2.tar.gz/download cmusphinx-de-ptm-voxforge-5.2.tar.gz
#model https://sourceforge.net/projects/cmusphinx/files/Acoustic%20and%20Language%20Models/German/cmusphinx-voxforge-de.lm.gz/download cmusphinx-voxforge-de.lm.gz
#model https://sourceforge.net/projects/cmusphinx/files/Acoustic%20and%20Language%20Models/German/cmusphinx-voxforge-de.dic/download cmusphinx-voxforge-de.dic
