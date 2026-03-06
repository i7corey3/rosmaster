#!/usr/bin/env bash
BASEDIR=$(cd $(dirname $0) && pwd)

    case $1 in
        update)
            
            echo Moving the files to their place
            echo
            echo Scripts are in /usr/local/bin/
            echo System services are in /etc/systemd/system/
            
            find "$BASEDIR/scripts" -type f -name "*.sh" -exec chmod +x {} +
            sudo cp -r $BASEDIR/scripts/* /usr/local/bin/
            for dir in $BASEDIR/services/*; do
                if [ -d "$dir" ]; then
                    for service in $dir/*; do
                        sudo cp $service /etc/systemd/system/
                    done
                fi
            done
          
            sudo systemctl daemon-reload
        ;;
        enable)
            echo Enabling all services!!
            echo

            for filename in $BASEDIR/services/*.service; do
                sudo systemctl enable $(basename s$filename)
            done
        ;;
        disable)
            
            echo Disabling all services!!
            echo

            for filename in $BASEDIR/services/*.service; do
                sudo systemctl disable $(basename s$filename)
            done
        ;;
        remove)
            echo Removing all services and sripts
            for filename in $BASEDIR/services/*.service; do
                sudo systemctl disable $(basename s$filename)
            done
            for filename in $BASEDIR/services/*.service; do
                if [ -f /etc/systemd/system/$(basename s$filename) ]; then
                    sudo rm -rf /etc/systemd/system/$(basename s$filename)
                    echo $(basename s$filename) removed
                fi
            done
            for filename in $BASEDIR/scripts/*.sh; do
                if [ -f /usr/local/bin/$(basename s$filename) ]; then
                    sudo rm -rf /usr/local/bin/$(basename s$filename)
                    echo $(basename s$filename) removed
                fi
            done

        ;;
        *)
            printf "The following commands are:
update
enable
disable
remove
"
            
        ;;
    esac
   
    

    