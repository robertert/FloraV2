#!/usr/bin/env bash
#
# Wrapper na idf.py: sam wciaga srodowisko ESP-IDF i wykrywa port szeregowy.
#
# Uzycie:  ./dev.sh <node|hub> <komenda> [dodatkowe argumenty do idf.py]
# Pomoc:   ./dev.sh help
#
# Zmienne srodowiskowe:
#   IDF_ACTIVATE  sciezka do activate_idf_vX.Y.sh (domyslnie v6.1)
#   FLORA_PORT    wymus port, np. /dev/cu.usbserial-0001
#   FLORA_BAUD    baudrate wgrywania, np. 921600

set -euo pipefail

IDF_ACTIVATE="${IDF_ACTIVATE:-$HOME/.espressif/tools/activate_idf_v6.1.sh}"
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR=""
PORT=""
IDF_PY_PYTHON=""
IDF_PY_SCRIPT=""
IDF_PORT_ARGS=()

die() {
	echo "$@" >&2
	exit 1
}

# Skrypt aktywacji EIM odmawia dzialania przez "source" wewnatrz skryptu
# (sprawdza $0), ale ma tryb -e, ktory wypisuje zmienne w formacie KEY=VALUE.
# To oficjalna droga nieinteraktywna i z niej korzystamy.
load_idf_env() {
	[ -f "$IDF_ACTIVATE" ] || die "Brak skryptu aktywacji ESP-IDF: $IDF_ACTIVATE
Podaj wlasna sciezke: IDF_ACTIVATE=/sciezka/activate_idf_vX.Y.sh $0 ..."

	local line key value tools_path=""
	while IFS= read -r line; do
		case "$line" in
			*=*) ;;
			*) continue ;;
		esac
		key="${line%%=*}"
		value="${line#*=}"
		case "$key" in
			PATH) tools_path="$value" ;;
			SYSTEM_PATH) ;; # wlasnego PATH nie nadpisujemy
			*) export "$key=$value" ;;
		esac
	done < <(bash "$IDF_ACTIVATE" -e)

	[ -n "$tools_path" ] && export PATH="$tools_path:$PATH"
}

activate_idf() {
	# Jesli srodowisko jest juz aktywne w tej powloce, nie ruszamy go.
	if [ -z "${IDF_PATH:-}" ] || [ -z "${IDF_PYTHON_ENV_PATH:-}" ]; then
		load_idf_env
	fi

	# idf.py jest funkcja powloki, nie binarka, wiec wolamy skrypt wprost
	# przez pythona z venva IDF (tak samo jak robi to funkcja z activate).
	IDF_PY_PYTHON="$IDF_PYTHON_ENV_PATH/bin/python"
	IDF_PY_SCRIPT="$IDF_PATH/tools/idf.py"

	[ -x "$IDF_PY_PYTHON" ] || die "Brak pythona z venva IDF: $IDF_PY_PYTHON"
	[ -f "$IDF_PY_SCRIPT" ] || die "Brak idf.py: $IDF_PY_SCRIPT"

	# idf.py marudzi, gdy locale nie obsluguje Unicode.
	if [ -z "${LC_ALL:-}" ] && [ -z "${LC_CTYPE:-}" ]; then
		export LC_CTYPE="en_US.UTF-8"
	fi
}

idf() {
	"$IDF_PY_PYTHON" "$IDF_PY_SCRIPT" -C "$PROJECT_DIR" "$@"
}

# Kandydaci na port: przejsciowki USB-serial oraz wbudowany USB-OTG (S2/S3/C3).
list_ports() {
	local p
	for p in /dev/cu.usbserial-* /dev/cu.usbmodem* /dev/cu.wchusbserial* /dev/cu.SLAB_USBtoUART*; do
		[ -e "$p" ] && echo "$p"
	done
	return 0
}

# Ustawia globalne $PORT albo konczy z bledem i podpowiedzia.
resolve_port() {
	if [ -n "${FLORA_PORT:-}" ]; then
		PORT="$FLORA_PORT"
		return
	fi
	if [ -n "${ESPPORT:-}" ]; then
		PORT="$ESPPORT"
		return
	fi

	local found=() p
	while IFS= read -r p; do
		found+=("$p")
	done < <(list_ports)

	case "${#found[@]}" in
		0)
			die "Nie znalazlem zadnego portu szeregowego.
Podlacz plytke kablem do portu danych i sprawdz: ./dev.sh ports"
			;;
		1)
			PORT="${found[0]}"
			echo "Port: $PORT"
			;;
		*)
			echo "Znalazlem wiecej niz jeden port:" >&2
			printf '  %s\n' "${found[@]}" >&2
			die "Wskaz jeden: FLORA_PORT=<port> $0 ..."
			;;
	esac
}

# Argumenty portu i baudrate dla komend, ktore gadaja z plytka.
port_args() {
	resolve_port
	IDF_PORT_ARGS=(-p "$PORT")
	if [ -n "${FLORA_BAUD:-}" ]; then
		IDF_PORT_ARGS+=(-b "$FLORA_BAUD")
	fi
}

usage() {
	cat <<'EOF'
Uzycie: ./dev.sh <node|hub> <komenda> [argumenty do idf.py]

Jednostki:
  node          glowica masztu
  hub           bryla (naped, zasilanie, podlewanie)

Komendy:
  build         zbuduj projekt
  flash         wgraj na plytke (port wykrywany automatycznie)
  monitor       podlacz konsole (wyjscie: Ctrl+])
  fm            build + flash + monitor, najczestszy cykl
  menuconfig    konfiguracja projektu
  size          raport zajetosci pamieci
  clean         usun artefakty builda
  fullclean     usun cale build/ (po zmianie targetu lub wersji IDF)
  reconfigure   przebuduj cmake bez kompilacji
  erase         wyczysc cala pamiec flash na plytce

Bez wyboru jednostki:
  ports         wypisz znalezione porty szeregowe
  shell         wypisz komende do aktywacji IDF w biezacej powloce
  help          ta pomoc

Cokolwiek innego jest przekazywane wprost do idf.py, np.:
  ./dev.sh hub build -v
  ./dev.sh hub partition-table

Zmienne: FLORA_PORT (wymus port), FLORA_BAUD (baudrate), IDF_ACTIVATE (inna wersja IDF)
EOF
}
# Komendy, ktore nie dotycza konkretnej jednostki.
case "${1:-help}" in
	help | -h | --help)
		usage
		exit 0
		;;
	ports)
		if list_ports | grep -q .; then
			list_ports
		else
			echo "Brak portow szeregowych."
		fi
		exit 0
		;;
	shell)
		echo "source $IDF_ACTIVATE"
		exit 0
		;;
esac

target="${1:-}"
case "$target" in
	node | hub)
		PROJECT_DIR="$REPO_DIR/$target"
		;;
	"")
		usage >&2
		exit 1
		;;
	*)
		die "Nieznana jednostka: $target
Dostepne: node, hub. Uzycie: $0 <node|hub> <komenda>"
		;;
esac
shift
[ -f "$PROJECT_DIR/CMakeLists.txt" ] || die "Brak projektu ESP-IDF w $PROJECT_DIR"

cmd="${1:-help}"
[ $# -gt 0 ] && shift

case "$cmd" in
	build)
		activate_idf
		idf build "$@"
		;;
	flash)
		activate_idf
		port_args
		idf "${IDF_PORT_ARGS[@]}" flash "$@"
		;;
	monitor)
		activate_idf
		port_args
		idf -p "$PORT" monitor "$@"
		;;
	fm | flash-monitor)
		activate_idf
		port_args
		idf "${IDF_PORT_ARGS[@]}" build flash monitor "$@"
		;;
	erase | erase-flash)
		activate_idf
		port_args
		idf "${IDF_PORT_ARGS[@]}" erase-flash "$@"
		;;
	menuconfig | size | clean | fullclean | reconfigure)
		activate_idf
		idf "$cmd" "$@"
		;;
	ports)
		if list_ports | grep -q .; then
			list_ports
		else
			echo "Brak portow szeregowych."
		fi
		;;
	shell)
		echo "source $IDF_ACTIVATE"
		;;
	help | -h | --help)
		usage
		;;
	*)
		activate_idf
		idf "$cmd" "$@"
		;;
esac
