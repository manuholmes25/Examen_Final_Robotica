# --- DOCKER_ROS2_SETUP_BLOCK ---
# Si ves esta linea, el script de inicializacion ya configuro este archivo.

source /opt/ros/humble/setup.bash
export TERM=xterm-256color

# Alias útiles
alias cb='colcon build --symlink-install'
alias s='source install/setup.bash'
alias rt='ros2 topic list'
alias rn='ros2 node list'

# Colores
alias ls='ls --color=auto'
alias grep='grep --color=auto'

# Prompt: (distro) usuario@ROBOT(rojo): ruta $
parse_git_branch() { git branch 2> /dev/null | sed -e '/^[^*]/d' -e 's/* \(.*\)/(\1)/'; }
PS1='\[\033[01;33m\](humble) \[\033[01;32m\]\u@\[\033[01;31m\]\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]$ '
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
