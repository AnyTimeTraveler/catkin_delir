# Delirium recognition

Eine Software zur Simulation eines Patienten, der auf ein Delir evaluiert werden soll.

## Installation und Start

Für den Start auf dem vorkonfigurierten Pi können Schritte 1 und 4 übersprungen werden.  
Unterpunkte 

1. Catkin_Workspace mit den Befehlen aus der Datei `create_catkin_workspace.sh` erstellen.  
   Ggf. müssen die Befehle auf das gewünschte System angepasst werden
2. In den Skripten `build.sh` und `start.sh` muss der Pfad des Catkin_Workspaces angepasst werden.
3. Aktuellen Ordner in `catkin_workspace/src/delir` kopieren
4. `build.sh` ausführen
5. IP des leistungsfähigen Linux-PCs für die Spracherkennung im Skript `start.sh` eintragen
6. Spracherkennung auf leistungsfähigen Linux-PC (oder VM, Docker-Container) im gleichen LAN installieren
   1. Ordner `voice_recognition_rs` dorthin kopieren
   2. In diesem Ordner das Skript `download_models.sh` ausführen
   3. IP des PI, Pfade der Spracherkennungsmodelle und eventuell weitere Parameter in der Datei `config.toml` anpassen
7. Das Skript `start.sh` starten.  
   Es sollte den Benutzer durch die nächsten Schritte des Starts leiten.
8. Wenn vom Skript `start.sh` aufgewordert, Spracherkennung starten
   1. Linux: Vorkompilierte Datei `voice_recognition_rs_linux_amd64` ausführen
   2. Nicht Linux oder zu altes/neues Linux: Bitte die Sektion "Anpassung und Kompilierung" lesen und anwenden.  
      Hierzu muss Rust installiert sein.  
      Installationsanweisungen: https://rustup.rs/



## Ordner-Struktur

 - catkin_delir : ROS Modul
   - launch : ROS Launchfile
   - scripts : Python Quellcode, der die Ergebnisse der Spracherkennung verarbeitet
   - voice_recognition_rs : Spracherkennung, die auf einem Computer mit genügend Rechenleistung ausgeführt werden sollte
   - build.sh : Skript, was das ROS-Paket baut
   - start.sh : Skript, was den Start des Projektes durch einen geleiteten Assistenten erleichtert
   - rust_audio_sender

### Spracherkennung (voice_recognition_rs)

Diese Software verbindet sich mit einem kleinen Hilfsprogramm namens `rust_audio_sender`,
welches auf dem Raspberry-Pi läuft.
Es empfängt das gesendete Audio und schickt es an eine oder mehrere Spracherkennungs-Bibliotheken.
(Es ist im Moment auf nur eine Spracherkennung gleichzeitig und auf Vosk optimiert)  
Der erkannte Text wird über einen TCP-Server abrufbar gemacht, mit dem sich das ROS-Package verbindet.

Man kann einige Konfigurations-Optionen in der Datei `config.toml` ändern.
Die Einstellungen sind dort jeweils dokumentiert.
Teile der Software können auch nicht mitkompiliert werden, wenn diese nicht nötig sind.
Die mitgelieferte ausführbare Datei wurde ohne Support für PocketSphinx kompiliert,
um kompatibilität mit Docker zu erhöhen.

Folgende Features können beim Kompilieren miteinbezogen oder ausgelassen werden:
 - **vosk**
   Erfordert die Verfügbarkeit der Vosk Bibliothek auf dem System
 - **pocketsphinx**
   Spracherkennung PocketSphinx  
   Erfordert die Verfügbarkeit der Pocketsphinx und CMUSphinx Bibliothek auf dem System
 - **noisefilter**  
   Erlaubt das Filtern des empfangenen Audiostreams nach Hintergrundgeräuschen.  
   Erfordert allerdings die Verfügbarkeit der Bibliothek "nnnoiseless" aus dem System (nicht gegeben bei Raspberry Pi) 
 - **localplayback**  
  Erlaubt das Zuhören zum empfangenen Audio, nachdem der Noisefilter (falls aktiviert) drübergelaufen ist.
  Erfordert allerdings eine Umgebung mit Soundsystem (was Docker ausschließt)

#### Anpassung und Kompilierung

Die Software kann auf Wunsch mit oder ohne diese Komponenten kompiliert werden.  
Beispiele für die Kompilierung:
```shell
# Um nur mit Vosk zu kompilieren:
cargo build --release --no-default-features --features vosk
# Um nur mit Pocketsphinx zu kompilieren:
cargo build --release --no-default-features --features pocketsphinx
# Um nur mit allen Features zu kompilieren:
cargo build --release --no-default-features --features vosk,pocketsphinx,noisefilter,localplayback
```

Die resultierende Datei liegt dann im Unterordner `target/release/voice_recognition_rs`
und muss vor dem Start in den Basisordner kopiert werden.
Alternativ kann das Programm auch direkt durch den Compiler gestartet werden:

```shell
# Um nur mit Vosk zu kompilieren und direkt zu starten:
cargo run --release --no-default-features --features vosk
```


### rust_audio_sender

Dieses Hilfsprogramm sendet Audio vom Mikrofon über das Netzwerk.
Es wird in der `start.sh` als Erstes gestartet.

### ROS-Package (delir)

Das ROS-Package delir übernimmt die Verarbeitung der erkannten Sprache.
Es wurde versucht, die technischen Details möglichst aus diesem Teil der Software fernzuhalten,
damit dieses Paket so einfach wie möglich in Zukunft angepasst werden kann.
Es besteht die Funkionalität, auf Schlüsselwörter warten, mit der Funktion `waitfor`.
Aktuell wird mit einer Nutzereingabe zur Kontrolle des Timings gearbeitet.
Wenn das zu buchstabierende Wort gewählt wird, fängt die Erkennung der Buchstabierung direkt an.
Vor der Erkennung der Logikfragen muss auch noch einmal mit Enter bestätigt werden.
Da dieses Packet in Python geschrieben ist, lassen sich diese Umstände einfach verändern, wenn gewünscht.

## Startreihenfolge

1. rust_audio_sender (wird von dem Skript `start.sh` übernommen)
2. voice_recognition_rs
3. ROS-Package
