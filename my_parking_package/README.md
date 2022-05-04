# Autonomes Einparken - Aufgabe 2

Dies ist das Template für die Aufgabe 2 des Praktikums Fahrzeuginformatik WiSe 2021/22.

## Aufgabenstellung

In der zweiten Aufgabe soll ein autonomes Parkmanöver für parallel zur Straße orientierte Parklücken (Längsparken) implementiert werden. Ihnen stehen bereits die Position des Parkplatzes und weitere, zur Implementierung notwendige, Informationen, zur Verfügung. Ferner erhalten Sie das Catkin-Paket **my_parking_package** als Template, in dem die Einbindung in das System bereits implementiert ist und Sie können sich somit beim Bearbeiten der Aufgabe auf die wesentlichen Aspekte des Parkmanövers konzentrieren.

### Vorbereitung

Kopieren Sie das Paket **my_parking_package** in den **src** Ordner Ihres Git-Repositorys. Bauen Sie anschließend das Workspace mit **catkin build** erneut, um zu sehen, ob das Template auf Ihrem System kompiliert.

    catkin_ws
        ├── src
        │   ├── gazebo_world_generator
        │   ├── gruppeX
        |   |   └── my_controller_package
        |   |   └── my_parking_package
        |   |       ├── src
        |   |       └── ...
        │   ├── isfl_extra_utils
        │   ├── ros
        │   └── simulation
        └── ...
**Abb. 1**: Ordnerstruktur nach dem Kopieren des Templates.

### Implementierung

Ihren Park-Algorithmus implementieren Sie in der Datei **my_parallel_parking_behavior.py**, welche Sie im Ordner **src/planning_stage** des Templates finden. In der Methode **plan_my_parallel_parking_behavior** stehen Ihnen die Position und Orientierung des Parkplatzes, Angaben zur Länge des Parkplatzes und des Fahrzeugs, sowie normalisierte Richtungsvektoren für Ihre Berechnungen zur Verfügung. Ihre Aufgabe besteht darin, Wegpunkte zu bestimmen, welche das Fahrzeug auf dem Weg in die Parklücke abfahren soll und den Ablauf mit Hilfe von States einer Statemachine zu implementieren.
Für diese Berechnungen stehen Ihnen mehrere Hilfsfunktionen in der Datei street.py zur Verfügung. Ihnen ist es jedoch auch freigestellt, eigene Interpolationsverfahren zu implementieren.

Die Regeln für den Carolo-Cup legen fest, dass ein Parkversuch als erfolgreich gewertet wird, wenn sich mindestens drei Räder vollständig im Parkbereich befinden. Weiterhin existiert eine Vorgabe zum Einsatz des Blinkers: Das Fahrzeug blinkt bei Anfahrt zur Lücke rechts und beim Ausfahren entsprechend links. Sobald das Parkmanöver abgeschlossen ist (das Fahrzeug also mit mindestens drei Reifen im Parkbereich steht) werden die Warnblinker eingeschaltet, um das Ende des Manövers zu signalisieren. Abweichungen von diesen Vorgaben werden mit Strafmetern geahndet. Ihre Implementierung sollte diese Vorgaben erfüllen und zuverlässig in einer mittelgroßen Lücke (ca. 60 cm Länge) einparken können. Weitere Informationen zu den geltenden Regeln erhalten Sie in den Regularien zum Carolo-Cup: 
https://www.tu-braunschweig.de/fileadmin/Redaktionsgruppen/Institute_Fakultaet_5/Carolo-Cup/Basic-Cup_Regulations.pdf (Seite 18 f.)

Die Wahl des Parkmanövers ist Ihnen freigestellt. Beachten Sie bei Ihrer Arbeit jedoch die Vorgaben aus dem Carolo-Cup und vergessen Sie bei der Implementierung nicht den Einsatz der Blinkern in der Statemachine zu hinterlegen.

### Abgabe / Präsentation

Für das Bestehen der Aufgabe müssen Sie mindestens ein Parkmanöver implementiert haben und die Lösung bei einer kurzen Demonstration erklären können. Bei der Präsentation müssen alle Gruppenmitglieder anwesend sein und einen Teil der Lösung erklären.
