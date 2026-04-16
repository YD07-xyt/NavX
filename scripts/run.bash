#!/bin/bash
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

screen -dmS serial bash -c "cd $DIR && ./serial.bash; exec bash"
#screen -dmS tf2 bash -c "cd $DIR && ./tf2.bash; exec bash"  
screen -dmS nav bash -c "cd $DIR && ./nav.bash; exec bash"  
sleep 1
screen -dmS bt_tree bash -c "cd $DIR && ./bt_tree.bash; exec bash"

echo "✅ 所有服务已启动，使用 'screen -ls' 查看"

tail -f /dev/null