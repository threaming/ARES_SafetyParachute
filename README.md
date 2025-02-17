# ARES Parachute Board

**PIN 1 BEI DEBUG HEADER AUF ROTE SEITE DES KABELS!!!!**

![Schema Board](docs/ares-parachute-board.svg)

## Projektstruktur
Das Projekt ist Standardmässig, wie von STMCubeMX vorgesehen, aufgesetzt, mit CMake als build.

Libraries sind in `Drivers/STM32F1xx_HAL_Driver` vorhanden.

### STM32CubeMX

Das Projekt wurde mit STM32CubeMX generiert. Dieses Programm generiert Code an verschiedensten Orten - weshalb Kommentare generiert werden, wo 'User Code' in Ordnung ist, und wo er überschrieben wird.

Schlussendlich ist STM32CubeMX jedoch auch nur ein Tool, und ein C Programm ein C Programm. Wenn also über STM32CubeMX keine neuen Konfigurationen generiert werden, kann man ruhig überall im Code etwas ändern.

## Wie setzt man die IDE (VSCode) auf?

- VS Code installieren
- STM32 Extension installieren
  - Relevant: Profile, siehe Post von Erich Styger hier: [MCUOnEclipse](https://mcuoneclipse.com/2023/11/11/vs-code-cure-the-extension-creep-with-profiles/)
- [STM32CubeCLT](https://www.st.com/en/development-tools/stm32cubeclt.html) installieren - wie von Extension gefordert
  - STM32CubeMX braucht man nicht umbedingt, ausser man will umbedingt Pinbelegungen mit GUI ändern.
- Für Flashing/Debugging: Treiber installieren (Standard für ARES: MCULink)
  - MCU-Link: OpenOCD installieren, Path in launch.json auf OpenOCD Path setzen.
    - Siehe Subsektion direkt unten für Installation
  - SEGGER: JLink tools installieren, Path in launch.json auf JLink install setzen bei JLINK debug

### Wie installiert man OpenOCD?

OpenOCD wird nicht über einen Installer installiert, sondern manuell zum Path hinzugefügt.

Damit OpenOCD im gesamten System verfügbar ist, sollte die untenstehende Anleitung befolgt werden. Alternativ können auch nur Schritte 1 und 2 befolgt werden, um dann den Pfad zu `OpenOCD/bin/openocd.exe` in `.vscode/launch.json` auf Zeile 12 zu spezifizieren (Zeile entkommentieren und Pfad anpassen).

#### 1. Herunterladen  
Lade die neueste OpenOCD-Version von [GitHub Releases](https://github.com/openocd-org/openocd/releases) herunter. Wähle die passende .tar.gz-Datei für Windows. Diese kann wie ein .zip enpackt werden.

#### 2. Entpacken  
Entpacke die .tar.gz-Datei in einen Ordner, z. B.:  
`C:\Tools\OpenOCD`

#### 3. Zum Systempfad hinzufügen  
1. Drücke `Win + R`, gib `sysdm.cpl` ein und drücke `Enter`.  
2. Gehe zu **Erweitert** → **Umgebungsvariablen**.  
3. Wähle unter **Systemvariablen** die Variable `Path` und klicke auf **Bearbeiten**.  
4. Klicke auf **Neu** und füge den `bin`-Ordner von OpenOCD hinzu, z. B.: `C:\Tools\OpenOCD\bin`
5. Klicke auf **OK**, um die Änderungen zu speichern.

#### 4. Testen  
Öffne ein **Terminal** (`Win + R`, `cmd`, `Enter`) und tippe:  

```sh
openocd --version
```

Du solltest etwas in dieser Art sehen:

```
Open On-Chip Debugger 0.12.0
Licensed under GNU GPL v2
For bug reports, read
  http://openocd.org/doc/doxygen/bugs.html
```

### ACHTUNG: ZU STM32CUBEMX

Gewisse STM32CubeMX Versionen haben Bugs, die z.B. das Linkerscript (STM32F103CBTx_FLASH.ld) kaputt machen, deswegen aufpassen, falls man doch STM32CubeMX verwenden will.

Version 6.13 scheint, zumindest auf Linux, defekt zu sein, deswegen wurde 6.12.1 verwendet, die jedoch einen Bug in der Linkerfile generation hat. Wenn Code regeneriert wird, über git die changes an `STM32F103CBTx_FLASH.ld` discarden, oder von `STM32F103CBTx_FLASH.ld.backup` wiederherstellen.

**WENN CHANGES MIT STM32CUBEMX 6.13 GEMACHT WERDEN, KANN DAS PROJEKT NICHT MEHR MIT ÄLTEREN VERSIONEN GEÖFFNET WERDEN!**

## Builden/Debuggen

Um das Projekt zu builden, kann das Kommando `CMake/Build` (Standardmässig auch `F7`) oder `CMake/Clean Rebuild` verwendet werden (mit `CTRL+SHIFT+P` kommt man in VSCode jederzeit zum Kommando-Menü).

Zum debuggen (beinhaltet auch Standard-Build), links auf das Play-Symbol mit dem 'Bug' drücken, falls nicht schon ausgewählt, die richtige Config auswählen (MCU-Link: `MCU-Link: CMSIS-DAP cortex-debug` auswählen), und auf den Startknopf drücken. Das geht standardmässig auch über `F5`, sobald die richtige Konfiguration ausgewählt ist.

## Hinzufügen von Source Files

Sollten neue Sourcedateien hinzugefügt werden:

Das Projekt buildet über `CMake`. `CMake` baut nur die Sources, die explizit angegeben werden, nicht automatisch alle Sources in einem Ordner - das ist präferiert, da so klarer festgelegt werden kann, was sich genau im Projekt befindet und was nicht. 

Bei einem `CMake` Projekt ist das 'Hauptfile', welches die Projektgenerierung regelt, `CMakeLists.txt`. Dieses befindet sich normalerweise im Hauptordner. Wenn man sich `CMakeLists.txt` im Projekt-Root anschaut, findet man, dass hier auf Zeile 49 ein `target_sources()` besteht, bei dem man User Sources hinzufügen kann, sowie ein `target_include_directories()`. Hier können Sources hinzugefügt werden. 

Als Beispiel: das File `test.c`, mit `test.h` soll zum Build hinzugefügt werden. `test.c` und `test.h` befinden sich in den gleichen Ordnern wie `main.c` und `main.h`, (`./Core/Src/test.c`, und `./Core/Inc/test.h`)

Um `test.c` hinzuzufügen, muss in `./CMakeLists.txt` in `target_sources()` folgendes hinzugefügt werden:

```CMake

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    ./Core/Src/test.c
)
```

`test.h` müsste ähnlich hinzugefügt werden, der Ordner `./Core/Inc` ist jedoch schon durch das automatisch generierte `CMakeLists.txt`, welches sich in `./cmake/stm32cubemx/CMakeLists.txt` befindet, schon inkludiert (Zeile 15):

```CMake
target_include_directories(stm32cubemx INTERFACE
    ../../Core/Inc
    ../../Drivers/STM32F1xx_HAL_Driver/Inc
    ../../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy
    ../../Drivers/CMSIS/Device/ST/STM32F1xx/Include
    ../../Drivers/CMSIS/Include
)
```

Somit sind alle Header-Files, die sich in `./Core/Inc` befinden, automatisch inkludiert. Beim Projektumfang in ARES ist es höchstwahrscheinlich nicht notwendig, zusätzliche Include-Folder zu definieren, also sollten benötigte Header-Files in diesen Ordner platziert werden.

## Funktion Demoprogramm

`printf()` output ist momentan rerouted auf SWO/ITM. Dies funktioniert mit der momentanen Konfiguration für den MCU-Link noch nicht - deswegen funktioniert `printf()` momentan nicht.

Der main loop wartet nach Exekution für 10ms und inkrementiert count als primitives 'scheduling'. 

~~Falls mit `#define ENABLE_TEMP 1` aktiviert, wird ca. jede Sekunde der Temperatursensor des Accelerometer am ersten I2C-Bus ausgelesen und über `printf()` ausgegeben.~~ Funktioniert momentan nicht mit MCU-Link

Falls mit `#define ENABLE_ACCEL_LED 1` aktiviert, wird der Zustand des Accelerometers jeden Loop ausgelesen, und falls die x, y bzw. z Register über einem bestimmten Wert sind, LED1, 2, bzw. 3 angeschaltet/ausgeschaltet.


### Note Debugging/SWO


Pin 1 für den Debugheader ist auf dem Board markiert -> Die rote Seite des Debug-Kabels muss auf dieser Seite eingesteckt werden.

Der Debugger speist das Board nicht, es muss extern über ein USB-C Kabel gespiesen werden.

Debug-Configs existieren für STLink (Untested), SEGGER J-Link (Alles funktioniert) und CMSIS-DAP/OpenOCD (Getestet mit MCU-Link von NXP, funktioniert alles bis auf SWO).

Damit die Debugs laufen, müssen in `.vscode/launch.json` die Pfäde angepasst werden:

- Generell muss die STM32 Extension wissen, wo sich STM32CubeCL befindet, damit der Path dazu gelesen werden kann
- Für SEGGER muss `"serverpath"` auf den Path zu `JLinkGDBServerCL.exe` gesetzt werden. Diese .exe sollte sich im J-Link install directory befinden.
- CMSIS-DAP sollte 'einfach funktionieren', solange OpenOCD auf dem Computer installiert ist. Wenn `openocd` nicht im PATH ist, muss eventuell `"serverpath"` noch auf den Pfad zu `openocd.exe` gesetzt werden

Um zu debuggen (Build, Connect, Flash, Attach GDB, Halt Main) kann dann die normale Debug-Funktion von VSCode angewendet werden (Käfer mit Startsymbol in der Linken Leiste). Oben links kann die Debugkonfiguration ausgewählt werden. Wenn die richtige Debug-Konfiguration ausgewählt ist, kann man entweder über den Startknopf, oder einfach über `F5` Builden, Flashen, Attachen und debuggen.


## Pinbelegung

`PB5` und `PB8` sind als Inputs definiert, `PB5` mit Pull-down, `PB8` mit Pull-up.

Falls die Inputs geändert werden müssen, kann man diese rekonfigurieren, entweder im Code oder mit STMCubeMX.

## Supercap-Speisung

Um das Board über den Supercap zu betreiben, muss sowohl dieser als auch JP1 installiert werden.

Wenn der mittlere und obere Pin von JP1 verbunden sind (mit einem Jumper) ist der Supercap mit der 5V-Rail, die über die USB-Buchse gespiesen wird, verbunden. Zwischen der USB-Buchse und der 5V-Rail befindet sich ein 500mA Stromlimiter, um nicht zu viel Strom vom USB-Port zu ziehen - wenn dieser Auslöst, leuchtet LED1. Sobald LED1 ausgeht, sollte das Board betriebsbereit sein - idealerweise lässt man es noch ein wenig länger am Strom hängen, um die letzten paar Prozent des Supercaps aufzuladen.
