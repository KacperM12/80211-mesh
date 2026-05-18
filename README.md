Projekt został przystosowany do uruchomienia w docker. Ze względu na brak środowiska graficznego (GUI) wewnątrz kontenera, skrypt został skonfigurowany tak, aby wynikowa topologia sieci była zapisywana bezpośrednio do pliku `wykres_topologii.png`.
Aby zbudować obraz na podstawie pliku `Dockerfile`, trzeba otworzyć wiersz poleceń w głównym katalogu projektu i wykonać:
docker build -t mesh-simulator .

Potem żeby uruchomić:
docker run --rm -v ${PWD}:/app mesh-simulator
