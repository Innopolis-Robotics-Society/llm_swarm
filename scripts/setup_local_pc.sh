# Увеличиваем UDP буферы ядра (от discovery storms)
sudo sysctl -w net.core.rmem_max=8388608 2>/dev/null
sudo sysctl -w net.core.rmem_default=8388608 2>/dev/null