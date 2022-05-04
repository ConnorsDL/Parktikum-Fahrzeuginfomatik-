# Object Detection - Aufgabe 4

Dies ist das Template für die Aufgabe 4 des Praktikums Fahrzeuginformatik WiSe 2021/22.

## Aufgabenstellung

In der vierten Aufgabe soll eine Objekterkennung für Sperrflächen und Zebrastreifen implementiert werden.

Das Bild der Kamera wurde mit einem Inverse Perspective Mapping bereits für die Erkennung vorbereitet und die Erkennung der Strecke ist bereits gegeben. Funktionen, um die Strecke entlang einer von Ihnen gewählten Spur zu durchlaufen und Informationen über Punkte, Tangenten, Normalen, oder andere zur Implementierung notwendigen Informationen abzufragen, stehen Ihnen bereits zur Verfügung. Des Weiteren bekommen Sie Funktionen um die Koordinaten eines Punktes den jeweiligen Pixeln im Bild zuzuordnen.

Ihren Algorithmus sollen Sie in dem Catkin-Paket **my_detection_package** implementieren, welches Sie als Template erhalten. In diesem Package ist die Einbindung in das System bereits implementiert und Sie können sich somit beim Bearbeiten der Aufgabe auf die wesentlichen Aspekte der Objekterkennung konzentrieren.

### Vorbereitung

Kopieren Sie das Paket **my_detection_package** in den **src** Ordner Ihres Git-Repositorys. Bauen Sie anschließend das Workspace mit **catkin build** erneut, um zu sehen, ob das Template auf Ihrem System kompiliert.

    catkin_ws
        ├── src
        │   ├── gazebo_world_generator
        │   ├── gruppeX
        |   |   └── my_controller_package
        |   |   └── my_parking_package
        |   |   └── my_obstacle_avoidance_package
        |   |   └── my_detection_package
        |   |       ├── src
        |   |       └── ...
        │   ├── isfl_extra_utils
        │   ├── ros
        │   └── simulation
        └── ...
**Abb. 1**: Ordnerstruktur nach dem Kopieren des Templates.

### Implementierung

Ihren Algorithmus zur Objekterkennung implementieren Sie in der Datei **MyDetector.cpp**, welche Sie im Ordner src des Templates finden. In der Methode **my_detection_algorithm** steht Ihnen ein **DetectorInput** als Eingabe zur Erkennung zur Verfügung, in der die Strecke bereits erkannt wurde. Mit dieser Eingabe können Sie einen Iterator erzeugen, um die Strecke entlang einer Spur zu durchlaufen und Punkte, Tangenten, Normalen oder andere Informationen an diesen Stellen abzufragen. Außerdem können Sie mit dem **DetectorInput** den zugehörigen Pixel im Bild für einen Punkt entlang der Strecke berechnen.

Ihre Aufgabe besteht darin, Sperrflächen und Zebrastreifen entlang der Strecke zu erkennen und diese so an das System weiterleiten, dass diese beim Erzeugen der finalen Karte berücksichtigt werden können. Stellen Sie sicher, dass bei einer Sperrfläche die korrekte Position, Länge und Breite an das System übergeben wird, damit sie in der Karte markiert werden kann und das Fahrzeug diese umfährt. Beim Erkennen eines Zebrastreifens müssen Sie die korrekte Position und Länge an das System übergeben. Stellen Sie außerdem sicher, dass ein Zebrastreifen nicht als Sperrfläche erkannt wird oder umgekehrt.

### Abgabe / Präsentation

Für das Bestehen der Aufgabe müssen Sie die Objekterkennung so implementieren, dass das Fahrzeug korrekt auf die Sperrflächen und Zebrastreifen in der Karte reagieren kann. Die Objekte müssen in der Karte markiert sein, Sperrflächen sollen erfolgreich umfahren werden und vor einem Zebrastreifen soll das Fahrzeug kurz stehen bleiben.
