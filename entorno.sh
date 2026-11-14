# Prepara la terminal para trabajar con el workspace INA_Strech_WS (Pequeños Muralistas).
# Uso (bash o zsh):   source entorno.sh
# Después:            mbuild            (compila)
#                     ros2 launch muralistas_bringup sim_mural.launch.py

_WS="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
export INA_STRECH_WS="$_WS"
export PYTHONNOUSERSITE=1   # evita que numpy 2 / OpenCV 5 de ~/.local se mezclen con ROS Humble

if [ ! -d "$_WS/.venv" ]; then
  echo "No existe $_WS/.venv. Créalo con:"
  echo "  python3 -m venv --system-site-packages $_WS/.venv"
  echo "  $_WS/.venv/bin/pip install 'mujoco==3.2.6' 'numpy<2' 'opencv-python-headless==4.10.0.84'"
  return 1 2>/dev/null || exit 1
fi
source "$_WS/.venv/bin/activate"

if [ -n "$ZSH_VERSION" ]; then
  source /opt/ros/humble/setup.zsh
  [ -f "$_WS/install/setup.zsh" ] && source "$_WS/install/setup.zsh"
else
  source /opt/ros/humble/setup.bash
  [ -f "$_WS/install/setup.bash" ] && source "$_WS/install/setup.bash"
fi

mbuild() {
  (cd "$INA_STRECH_WS" && "$VIRTUAL_ENV/bin/python" -m colcon build --symlink-install \
      --cmake-args -DPython3_EXECUTABLE="$VIRTUAL_ENV/bin/python" "$@") || return 1
  if [ -n "$ZSH_VERSION" ]; then source "$INA_STRECH_WS/install/setup.zsh"; else source "$INA_STRECH_WS/install/setup.bash"; fi
}

echo "Workspace INA_Strech_WS listo: $_WS  (después de mbuild, si ros2 no ve un paquete nuevo: source entorno.sh)"
