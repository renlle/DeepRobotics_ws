#!/usr/bin/env bash
# nohup 常驻启动离线接口和算法 mock callback，不依赖 ROS/Conda。
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
cd "$ROOT"
PYTHON="${PYTHON_BIN:-python3}"
HOST="${OFFLINE_HOST:-127.0.0.1}"
PORT="${OFFLINE_PORT:-2335}"
MOCK_HOST="${OFFLINE_MOCK_HOST:-127.0.0.1}"
MOCK_PORT="${OFFLINE_MOCK_PORT:-2336}"
RUN_DIR="$ROOT/tmp/offline_rebuild_jobs"
while (($#)); do
  case "$1" in
    --host|--port|--mock-host|--mock-port)
      (($# >= 2)) || { echo "缺少参数值: $1" >&2; exit 2; }
      case "$1" in
        --host) HOST="$2" ;; --port) PORT="$2" ;;
        --mock-host) MOCK_HOST="$2" ;; --mock-port) MOCK_PORT="$2" ;;
      esac
      shift 2 ;;
    --help|-h)
      echo '用法: start_offline_rebuild_map.sh [--host IP] [--port 2335] [--mock-host IP] [--mock-port 2336]'
      exit 0 ;;
    *) echo "未知参数: $1" >&2; exit 2 ;;
  esac
done
for port in "$PORT" "$MOCK_PORT"; do
  [[ "$port" =~ ^[0-9]{1,5}$ ]] && ((10#$port >= 1 && 10#$port <= 65535)) || {
    echo "无效端口: $port" >&2; exit 2;
  }
done
[[ "$PORT" != "$MOCK_PORT" ]] || { echo '重建接口与算法 mock 必须使用不同端口' >&2; exit 2; }
mkdir -p "$RUN_DIR"
exec 9>"$RUN_DIR/start.lock"
flock 9

probe() {
  "$PYTHON" - "$1" "$2" "$3" "$ROOT" <<'PY'
import json, socket, sys
from urllib.request import build_opener, ProxyHandler
host, port, service, root = sys.argv[1:]
host = {"0.0.0.0": "127.0.0.1", "::": "::1"}.get(host, host)
try:
    with socket.create_connection((host, int(port)), timeout=0.5):
        pass
except ConnectionRefusedError:
    sys.exit(1)
except OSError as exc:
    print(str(exc), file=sys.stderr)
    sys.exit(2)
try:
    address = f"[{host}]" if ":" in host else host
    with build_opener(ProxyHandler({})).open(f"http://{address}:{port}/health", timeout=0.5) as response:
        data = json.load(response)
    if data.get("service") != service or data.get("workspace") != root:
        raise ValueError("端口上的服务不属于本项目")
    print(int(data["pid"]))
except Exception as exc:
    print(f"端口 {port} 已被占用或服务未就绪: {exc}", file=sys.stderr)
    sys.exit(2)
PY
}

start_service() {
  local name="$1" entry="$2" host="$3" port="$4" service="$5" pid rc current
  if pid="$(probe "$host" "$port" "$service")"; then
    printf '%s\n' "$pid" > "$RUN_DIR/$name.pid"
    printf '%s 已运行，PID=%s，端口=%s\n' "$name" "$pid" "$port"
    return
  else
    rc=$?
    ((rc == 1)) || return "$rc"
  fi
  # 子进程关闭启动锁描述符，保证脚本退出后可以再次执行。
  nohup "$PYTHON" -u "$ROOT/debug_md/offline_rebuild_map/$entry" \
    --host "$host" --port "$port" >> "$RUN_DIR/$name.log" 2>&1 < /dev/null 9>&- &
  pid=$!
  for ((i=0; i<50; i++)); do
    if ! kill -0 "$pid" 2>/dev/null; then
      echo "$name 启动失败，请查看 $RUN_DIR/$name.log" >&2
      return 1
    fi
    if current="$(probe "$host" "$port" "$service" 2>/dev/null)" && [[ "$current" == "$pid" ]]; then
      printf '%s\n' "$pid" > "$RUN_DIR/$name.pid"
      printf '%s 已通过 nohup 启动，PID=%s，端口=%s\n' "$name" "$pid" "$port"
      return
    fi
    sleep 0.1
  done
  echo "$name 健康检查超时，PID=$pid；请查看 $RUN_DIR/$name.log" >&2
  return 1
}

start_service mock_callback mock_offline_callback.py "$MOCK_HOST" "$MOCK_PORT" offline_rebuild_mock_callback
start_service server offline_rebuild_api.py "$HOST" "$PORT" offline_rebuild_map
echo "算法 mock callback 占用端口 $MOCK_PORT：http://$MOCK_HOST:$MOCK_PORT/callback"
echo "离线重建接口：http://$HOST:$PORT/api/offline/rebuild"
echo "日志和 PID 文件：$RUN_DIR"
