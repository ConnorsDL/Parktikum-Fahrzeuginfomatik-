# Regelung / Fahrzeugführung - Aufgabe 1

Dies ist das Template für die Aufgabe 1 des Praktikums Fahrzeuginformatik WiSe 2021/22.

## Aufgabenstellung

In der ersten Aufgabe soll die Fahrzeugführung des Fahrzeugs implementiert werden. Mit der Spur und der Fahrzeugposition als Eingabe sollen kontinuierlich Lenk Befehle errechnet werden, um das Fahrzeug entlang der Spur zu steuern.
Hierzu steht Ihnen das Catkin Paket **my_controller_package** als Template zur Verfügung, in dem die Einbindung in das System bereits implementiert ist und Sie sich beim Bearbeiten der Aufgabe nur auf die wesentlichen Aspekte der Fahrzeugführung konzentrieren können.

### Vorbereitung

Nachdem Sie das Projekt des Löwen Teams auf Ihrem System eingerichtet haben, laden Sie das Git-Repository Ihrer Gruppe in den **src** Ordner Ihres Catkin Workspaces und kopieren das Paket **my_controller_package** dort hinein. Bauen Sie anschließend das Workspace mit **catkin build** erneut um zu sehen, ob das Template auf Ihrem System kompiliert.

    catkin_ws
        ├── src
        │   ├── gazebo_world_generator
        │   ├── gruppeX
        |   |   └── my_controller_package
        |   |       ├── src
        |   |       └── ...
        │   ├── isfl_extra_utils
        │   ├── ros
        │   └── simulation
        └── ...
**Abb. 1**: Ordnerstruktur nach dem Kopieren des Templates.

### Implementierung

Ihren Algorithmus zur Fahrzeugführung implementieren Sie in der Datei **MyController.cpp** die Sie im Ordner **src** des Templates finden. In der Methode **calculate_drive_control** bekommen Sie die Spur und die Position des Fahrzeugs als Eingabe und müssen aus diesen Daten eine Geschwindigkeit und den Lenkwinkel der Räder berechnen, die als Ausgabe weiter an den Controller des Fahrzeugs gesendet werden.

Die Spur wird durch eine Trajektorie beschrieben, der das Fahrzeug folgen soll. Eine Trajektorie besteht aus mehreren Punkten, die Informationen wie die x und y Position des Punktes ausgehend vom Auto oder die maximale Geschwindigkeit an dieser Position enthalten.

Bei der Position des Autos handelt es sich um eine geschätzte Position 150 ms in der Zukunft. Dies hat den Hintergrund, dass auf der Hardware eine Verzögerung von 150 ms zwischen dem Senden eines Befehls und der Ausführung existiert. Dies wird auch in der Simulation simuliert, damit diese der Hardware möglichst ähnlich ist.

Informieren Sie sich, welche Regelungsverfahren es für Fahrzeuge gibt und implementieren Sie verschiedene Methoden um diese zu vergleichen. Als Einstieg können Sie sich den Pure Pursuit und den Stanley Algorithmus für Vorderachslenkung anschauen. Sie können aber auch kreativ werden und sich eine eigene Regelung überlegen und diese implementieren.

### Abgabe / Präsentation

Für das Bestehen der Aufgabe müssen Sie mindestens ein Regelverfahren implementiert haben und die Lösung bei einer kurzen Demonstration erklären können. Bei der Präsentation müssen alle Gruppenmitglieder anwesend sein und einen Teil der Lösung erklären.
