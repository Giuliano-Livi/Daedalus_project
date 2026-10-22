#!/usr/bin/env bash
# Avvia l'ambiente: PX4+Gazebo (container), agent DDS, GUI sul PC, shell ROS 2.
# Uso: scripts/start.sh [start|stop|restart|status] [--no-gui] [--no-shell]
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
GUI_PID_FILE="/tmp/asp_gz_gui.pid"
GUI_LOG="/tmp/asp_gz_gui.log"
GUI_CONFIG="gazebo/gui/gui.config"   # layout GUI con la finestra della depth camera
WORLD_FILE="gazebo/worlds/warehouse_px4.sdf"
WAIT_TIMEOUT=180

# Devono coincidere con quanto impostato in docker-compose.yml
export GZ_PARTITION="warehouse"
export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH:-$HOME/px4_gz/models:$HOME/px4_gz/worlds}"

log()  { echo -e "\033[1;32m[asp]\033[0m $*"; }
warn() { echo -e "\033[1;33m[asp]\033[0m $*" >&2; }
die()  { echo -e "\033[1;31m[asp]\033[0m $*" >&2; exit 1; }

cd "$PROJECT_DIR"

check_deps() {
  command -v docker >/dev/null || die "docker non trovato."
  docker compose version >/dev/null 2>&1 || die "docker compose non disponibile."
  [[ -f docker-compose.yml ]] || die "docker-compose.yml non trovato in $PROJECT_DIR."
  # Se manca il file, Docker creerebbe una cartella al suo posto e il sim non partirebbe
  [[ -f "$WORLD_FILE" ]] || die "Manca $WORLD_FILE (rigenerarlo con scripts/make_px4_world.py)."
}

wait_for_world() {
  log "Attendo che Gazebo carichi il world (max ${WAIT_TIMEOUT}s)..."
  local start=$SECONDS cid running topics recent
  while (( SECONDS - start < WAIT_TIMEOUT )); do
    cid="$(docker compose ps -q sim || true)"
    running="false"
    if [[ -n "$cid" ]]; then
      running="$(docker inspect -f '{{.State.Running}}' "$cid" 2>/dev/null || echo false)"
    fi
    if [[ "$running" != "true" ]]; then
      docker compose logs sim --tail 30 || true
      die "Il container sim si e' fermato."
    fi
    recent="$(docker compose logs sim --tail 300 2>&1 || true)"
    if grep -q "Failed to load a world" <<<"$recent"; then
      grep -E "\[Err\]" <<<"$recent" | tail -5 || true
      die "Gazebo non e' riuscito a caricare il world."
    fi
    if command -v gz >/dev/null; then
      topics="$(timeout 5 gz topic -l 2>/dev/null || true)"
      if grep -q "^/world/warehouse_px4/" <<<"$topics"; then
        log "World pronto."
        return 0
      fi
    elif grep -q "Gazebo world is ready" <<<"$recent"; then
      log "World pronto."
      return 0
    fi
    sleep 2
  done
  die "Timeout: controlla con docker compose logs sim --tail 40"
}

wait_for_vehicle() {
  local start=$SECONDS topics
  if ! command -v gz >/dev/null; then
    warn "gz non trovato sul PC: non posso verificare lo spawn del drone."
    return 0
  fi
  log "Attendo che PX4 inserisca il drone (max ${WAIT_TIMEOUT}s)..."
  while (( SECONDS - start < WAIT_TIMEOUT )); do
    topics="$(timeout 5 gz topic -l 2>/dev/null || true)"
    if grep -Eq '^/world/warehouse_px4/model/x500_depth_0/|^/model/x500_depth_0/' <<<"$topics"; then
      log "Drone pronto."
      return 0
    fi
    sleep 2
  done
  die "Timeout: il drone non e' apparso; controlla docker compose logs sim."
}

start_gui() {
  if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
    warn "Nessun display disponibile: salto la GUI."; return 0
  fi
  command -v gz >/dev/null || { warn "gz non trovato sul PC: salto la GUI."; return 0; }
  if [[ ! -d /opt/px4-gazebo/share/gz/models ]]; then
    warn "Manca /opt/px4-gazebo/share/gz (collegamento a ~/px4_gz): la GUI non trovera' le mesh."
  fi
  if [[ -f "$GUI_PID_FILE" ]] && kill -0 "$(cat "$GUI_PID_FILE")" 2>/dev/null; then
    log "GUI gia' attiva."; return 0
  fi
  setsid nohup env -u GTK_EXE_PREFIX -u GTK_MODULES -u GTK_PATH -u GIO_MODULE_DIR \
    gz sim -g --gui-config "$PROJECT_DIR/$GUI_CONFIG" >"$GUI_LOG" 2>&1 &
  echo $! > "$GUI_PID_FILE"
  log "GUI avviata (log: $GUI_LOG)."
}

allow_x11() {
  # finestre dal container dev (es. RViz2): consente l'X server solo a root locale
  if [[ -n "${DISPLAY:-}" ]] && command -v xhost >/dev/null; then
    xhost +SI:localuser:root >/dev/null 2>&1 || warn "xhost fallito: RViz2 nel container non potra' aprire finestre."
  fi
}

stop_gui() {
  if [[ -f "$GUI_PID_FILE" ]]; then
    kill -- "-$(cat "$GUI_PID_FILE")" 2>/dev/null || true
    rm -f "$GUI_PID_FILE"
  fi
}

cmd_start() {
  check_deps
  log "Avvio sim e agent..."
  docker compose up -d sim agent
  wait_for_world
  wait_for_vehicle
  (( GUI )) && start_gui
  allow_x11
  if (( OPEN_SHELL )); then
    log "Apro la shell ROS 2 (container dev). Per fermare tutto: scripts/start.sh stop"
    exec docker compose run --rm dev bash
  else
    log "Tutto avviato. Shell ROS 2: docker compose run --rm dev bash"
  fi
}

cmd_stop() {
  log "Chiudo GUI e container..."
  stop_gui
  docker compose down
}

cmd_status() {
  docker compose ps
  if command -v gz >/dev/null; then
    timeout 5 gz topic -e -t /stats -n 1 2>/dev/null | grep real_time_factor \
      || warn "Nessun dato da /stats (server non raggiungibile?)"
  fi
}

CMD="${1:-start}"; shift || true
GUI=1; OPEN_SHELL=1
for a in "$@"; do
  case "$a" in
    --no-gui)   GUI=0 ;;
    --no-shell) OPEN_SHELL=0 ;;
    *) die "Opzione sconosciuta: $a" ;;
  esac
done

case "$CMD" in
  start)   cmd_start ;;
  stop)    cmd_stop ;;
  restart) cmd_stop; cmd_start ;;
  status)  cmd_status ;;
  *) die "Comando sconosciuto: $CMD (start|stop|restart|status)" ;;
esac
