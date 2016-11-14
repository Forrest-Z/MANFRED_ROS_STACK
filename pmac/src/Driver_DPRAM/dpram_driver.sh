#!/bin/sh
module="dpram_driver"
device="dpram_driver"
mode="777"

binarioDriver="dpram_driver.ko"

if [ -e $binarioDriver ]; then    
    mv dpram_driver.ko Binario_driver/dpram_driver.ko
	rm -r .tmp_versions
	rm dpram_driver.mod.c
	rm dpram_driver.mod.o
	rm dpram_driver.o
	rm Module.symvers
	rm modules.order
	rm .dpram_driver.ko.cmd
	rm .dpram_driver.mod.o.cmd
    rm .dpram_driver.o.cmd
fi

# Group: since distributions do it differently, look for wheel or use staff
if grep -q '^staff:' /etc/group; then
    group="staff"
else
   group="wheel"
fi
# invoke insmod with all arguments we got
# and use a pathname, as insmod doesn't look in . by default
/sbin/insmod -f `pwd`/Binario_driver/${module}.ko $* || exit 1
# Recuperar el major number del driver instalado, asignado dinámicamente, por el S.O.
# Se extrae del fichero /proc/devices usando el comando de procesado de texto 'awk'.
# NOTA: SE DEBE INSTALAR GAWK desde el synaptic.
major=$(awk "\$2==\"${module}\" {print \$1}" /proc/devices)
# Imprimir major number para verificar que todo ha ido bien.
# Comprobar este número con el que podemos ver usando 'tail -f /var/log/syslog'
echo "dpram_driver con major number: ${major}"
# Borrar cualquier referencia antigua a al dispositivo $device en el directorio '/dev'
rm -f /dev/${device}
# Crear el dispositivo $device de nuevo
mknod /dev/${device} c ${major} 0
# Enlace simbolico. Origen del enlace simbolico: /dev/$device, destino del enlace simbolico: $device
ln -sf /dev/${device} ${device}
# Cambio de permisos en $device. Estos permisos son aplicables sólo a $device, conservando /dev/$device los que
# tuviera en el momento de crearse, que probablemente seran: -rwx------, es decir, solo el root puede acceder a
#/dev/$device.
chgrp ${group} ${device}
chmod ${mode}  ${device}

# Se envía el comando GATHER para poder activar el 'Backgroung Fixed Data' en la DPRAM.
chmod +x ../../bin/binComandos
../../bin/binComandos
