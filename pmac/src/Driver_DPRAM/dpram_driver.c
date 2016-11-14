#include <asm/uaccess.h>
// #include <linux/config.h> cambia a #include <linux/autoconf.h>
// En la version 10.04 de Ubuntu (y anteriores hasta la 7.04) el archivo autoconf.h
// estaba situado en "/usr/src/linux-headers-W.X.YY-ZZ-generic/include/linux/autoconf.h".
// En la version 10.10 de Ubuntu el archivo "autoconf.h" pasa a estar en la ruta
// "/usr/src/linux-headers-W.X.YY-ZZ-generic/include/generated/autoconf.h".
#include <generated/autoconf.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/fcntl.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/kdev_t.h>
#include <linux/kernel.h>
#include <linux/module.h>
// Permite obtener paametros de la linea de comandos en tiempo de carga del driver
// Ejemplo: insmod modulo_hello_p howmany=10 whom="Mom"
#include <linux/moduleparam.h>
#include <linux/pci.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/version.h>


#include <linux/proc_fs.h>
#include <asm/system.h>


 /** Linux permite extender la funcionalidad del núcleo en tiempo de ejecucion.
   * Cada pieza de codigo de este tipo se denomina modulo cargable o simplemente modulo.
	* La mayoria de los modulos son manejadores de dispositivo o drivers.
	* Un modulo es codigo objeto que se enlaza dinamicamente al nucleo mediante el 
	* mandato insmod (o modprobe).
	* Se desenlaza mediante el mandato rmmod.
	*/
   MODULE_LICENSE("Dual BSD/GPL");
   MODULE_AUTHOR("Juan Francisco Rascon Crespo");
   
 /** Los dispositivos se acceden a traves de nodos del sistema de ficheros, convencionalmente
   * ubicados en el directorio /dev
	* Hay dispositivos de caracteres y de bloques, identificados con una 'c'o una 'b' en
	* los permisos del nodo.
	* Mirar en el terminal haciendo: ls -l /dev
	*/
	
   int dpramInit(void);
   void dpramExit(void);      
   int dpramOpen(struct inode *inode, struct file *filp);
   int dpramClose(struct inode *inode, struct file *filp);	
   loff_t dpramLseek(struct file *filp, loff_t offset, int mode);
   ssize_t dpramRead(struct file *filp, char *buff, size_t count, loff_t *offset);
   ssize_t dpramWrite(struct file *filp, const char *buff, size_t count, loff_t *offset);	

  
/* Estructura que declara las funciones de acceso a ficheros */
   struct file_operations dpramdev_fops={    
   .owner=THIS_MODULE,
   .open=dpramOpen,
   .release=dpramClose,  
   .write=dpramWrite,   
   .read=dpramRead,
   .llseek=dpramLseek,
   };

// DEFINICION DE ESTRUCTURAS.
   struct DPRAM_DEV {
   // Numero principal, asociado al driver.
      int major;
   // Numero secundario, minor. Indice de dispositivo asociado al driver.       
      int minor;
   // Las direcciones fisicas son del tipo 'unsigned long'.      
   // Direccion virtual de la dpram 
      unsigned long vadd_dpram;             
   // Tamaño de la DPRAM      
      int tamano;
   // Offset de escritura y lectura       		
      unsigned long offset;             
   // Una variable definida como 'long int' ocupa 4 Bytes,
   // el mismo tamaño que un int (a secas).   
   // unsigned long *addr;
   // Creo que con esta sentencia el driver puede funcionar para arquitecturas de 64 bits.
      __u32 *addr;
      struct cdev cdev;
   };
   
   struct DPRAM_DEV dpramdev;
   struct pci_dev *pcidev = NULL;


		
   int dpramInit(void){
    
   // int check_mem = 0;
   // int err = 0;
   // int result= 0;
      
      dev_t devn = 0;
      
      struct pci_dev *pcidev = NULL;
   
   // Inicializar estructura 'dpramdev'.   
      memset(&dpramdev, '\0', sizeof(struct DPRAM_DEV));   
   // Obtener los numeros principal (major) y secundario (minor).
      if( alloc_chrdev_region(&devn, 0, 1, "dpram_driver") < 0 ) {
         printk(KERN_ALERT "DPRAM error: No se puede registrar la dpram.\n");		
         return -1;
      }   
      dpramdev.major = MAJOR(devn);	
      dpramdev.minor = MINOR(devn);
      printk(KERN_ALERT "DPRAM instalada, conectada al major device: %d\n", dpramdev.major);
   // El núcleo utiliza estructuras de tipo struct cdev para representar internamente los 
   // dispositivos de caracteres, es decir, los /dev/<lo_que_sea>. 
   // La estructura 'struct cdev' referencia el vector de
   // operaciones del driver. Es preciso asignar e inicializar una de estas estructuras por
   // cada dispositivo que soporta el driver.   
   // 'cdev_init' recibirá una estructura cdev vacia, inicializándola con la estructura 'file_operations'  pasada 
   // como segundo argumento.      
      cdev_init(&(dpramdev.cdev), &dpramdev_fops);
      dpramdev.cdev.owner = THIS_MODULE;
   // No me queda claro si esta instruccion esta de sobra.   
      dpramdev.cdev.ops = &dpramdev_fops;
      if (cdev_add (&(dpramdev.cdev), devn, 1) < 0){
         printk(KERN_ALERT "ERROR: Fallo al ejecutar la funcion 'cdev_add'.\n");  
      // Liberar los números de dispositivo antes de descargar el modulo.
         unregister_chrdev_region(devn, 1);
         return -1;
      }   
   // Configuracion del dispositivo PCI.
   // Buscamos el dispositivo por fabricante e identificador: Mirar el valor de estos numeros en /proc/devices
      pcidev = pci_get_device(0x1172, 0x0001, NULL);
      if(pcidev == NULL){
         printk(KERN_ALERT "ERROR: No existe el dispositivo PCI con fabricante: %#x, e identificador: %#x\n", 0X1172, 0X0001);
         cdev_del(&(dpramdev.cdev));
      // Liberar los números de dispositivo antes de descargar el modulo.
         unregister_chrdev_region(devn, 1);
         return -1;
         
      }
   // Habilitamos la comunicacion con el dispositivo PCI, es decir la PMAC.
   // This function actually enables the device. It wakes up the device and in some
   // cases also assigns its interrupt line and I/O regions. 
      pci_enable_device(pcidev);
   // Preguntar al kernel donde comienza el espacio de direcciones que tiene este periferico
   // asignado en la RAM del PC. A día de hoy, 4/11/2009 este espacio de direcciones comienza
   // en febf0000, hasta febfffff, es decir, 64K direcciones de memoria, es decir 64 * 1024 = 65536
   // direcciones de memoria. Cada dirección de memoria contiene un byte de datos. Todo lo anterior
   // se puede mirar con el comando 'sudo cat /proc/iomem'
      dpramdev.vadd_dpram = pci_resource_start(pcidev, 0);
      dpramdev.tamano = pci_resource_len(pcidev, 0);
   // 16 KB de memoria reseravados.
   // dpramdev.tamano = 0x4000;
      printk("Direccion base: dpramdev.vadd_dpram: %#lX.\n", dpramdev.vadd_dpram); 
      printk("Tamano: %#X.\n", dpramdev.tamano);       
   // Reservamos el espacion del kernel para nuestra tarjeta y que no lo ocupe otro dispositivo 
      if( (request_mem_region(dpramdev.vadd_dpram, dpramdev.tamano, "dpram_driver")) == NULL ){
         pci_dev_put(pcidev);  
      // Prohibida esta instruccion, o de lo contrario no se puede descargar el modulo 'driver_dpram_2010'
      // pci_disable_device (pcidev);             
         cdev_del(&(dpramdev.cdev));
      // Liberar los números de dispositivo antes de descargar el modulo.
         unregister_chrdev_region(devn, 1);
      }
   // Pasamos de la memoria fisica a direcciones virtuales
   // Si esto no de hace aparecera un fallo de segmentacion.
   // Re-map an arbitrary physical address space into the kernel virtual
   // address space. Needed when the kernel wants to access physical
   // memory directly.
      dpramdev.addr = ioremap(dpramdev.vadd_dpram, dpramdev.tamano);     
      printk("dpramdev.addr = %p.\n", dpramdev.addr);
      return 0;
   }
   
	
/* The cleanup function is used to handle initialization failures as well.
 * Thefore, it must be careful to work correctly even if some of the items
 * have not been initialized
 */
   void dpramExit(void){		
      dev_t devn = MKDEV(dpramdev.major, dpramdev.minor);       
      printk(KERN_ALERT "Descargando el driver dpram_driver.\n");
      iounmap(dpramdev.addr);
      release_mem_region (dpramdev.vadd_dpram, dpramdev.tamano);
   // Dentro de la función 'cdev_del' se hace un 'cdev_unmap', asi que no es necesario hacerlo
   // de forma explícita.
      cdev_del(&dpramdev.cdev);
   // Liberar los números de dispositivo antes de descargar el modulo.
      unregister_chrdev_region(devn, 1);
   // Must be called when a user of a device is finished with it.
   // When the last user of the device calls this function, the memory of the device is freed.         
      pci_dev_put(pcidev);      
   // Prohibida esta instruccion, o de lo contrario no se puede descargar el modulo 'driver_dpram_2010'
   // pci_disable_device (pcidev);
   
   }
   
	
   int dpramOpen(struct inode *inode, struct file *filp){
      struct DPRAM_DEV *dpramdev = NULL;
      dpramdev = container_of(inode->i_cdev, struct DPRAM_DEV, cdev);
      filp->private_data = dpramdev; 
      return 0;
   }
   
	
   int dpramClose(struct inode *inode, struct file *filp){
      return 0;   
   }
   
   
/** Ojito con esto:  
  * typedef long long               __kernel_loff_t;
  * typedef __kernel_loff_t         loff_t;
  * Asi que loff_t es de tipo long long --> 64 bits!!
  * Para imprimir un numero long long en formato decimal: %lld.
  * Para imprimir un numero long long en formato hexadecimal: %llX.
  */
   loff_t dpramLseek(struct file *filp, loff_t offset, int mode){   
   // printk("dpram_lseek:  %016llx(hex).\n", offset);
   // printk(KERN_ALERT "dpram_lseek: %#llX.\n", offset);
   // Lo utilizamos para posicionarnos dentro de la memoria
      struct DPRAM_DEV *dpramdev = filp->private_data;
      loff_t newpos = 0;
      switch(mode){
         case 0: /* SEEK_SET*/
            newpos = offset;
            break;
         case 1: /* SEEK_CUR*/
            newpos = filp->f_pos + offset;
            break;
         case 2: /* SEEK_END*/
            newpos = dpramdev->tamano + offset;
            break;
         default: /* No puede pasar*/
            return -EINVAL;
      }
      if(newpos < 0){
         return -EINVAL;
      }
      filp->f_pos = newpos;
      return newpos;
   }
   
	
   ssize_t dpramRead(struct file *filp, char *buff, size_t count, loff_t *offset){
      int size = 0;
   // int off = *offset;
   // Comprobamos si la memoria de usuario escogida cae dentro de la memoria de kernel
   // si es asi devolvemos un fallo.
      if(access_ok(VERIFY_WRITE, buff, count)== 0){ 
         printk(KERN_ALERT "dpram_read: Fallo al acceder a la memoria de usuario.\n");
         return -EFAULT;  
      }
      // printk("direccion2 [%p] tamaño [%d]\n",s.addr,count);
      // Copiamos a la memoria de usuario el segmento de la memoria DPRAM ///
      // Mirar pag 8 Manual DPRAM. Explicitamente indica:
      // host_addres = host_start_address + 4 x (PMAC_Address - $D000)
      // Como dpramdev.addr es un puntero a tipo de datos __u32 (32 bits, i.e, 4 bytes)
      // cada vez que sumamos una unidad a dicha direccion de memoria, en realidad se
      // esta sumando 4 direcciones (asi es la aritmetica de punteros).
      // Por tanto para reproducir la instruccion de la pagina 8 del manual de la DPRAM solo
      // es necesario hacer: dpramdev.addr + filp->f_pos, ya que esto en realidad equivale a
      // dpramdev.addr + 4*filp->f_pos. Por supuesto que flip->f_pos tendra el valor adecuado, i.e,
      // flip->f_pos = PMAC_Address - $D000
      size = copy_to_user(buff, dpramdev.addr + filp->f_pos, count); 
      printk("buff: %s.\n", buff);
      printk(KERN_ALERT "dpramdev.addr = %p.\n", dpramdev.addr);      
      printk(KERN_ALERT "filp->f_pos: %#llX.\n", filp->f_pos);
      printk(KERN_ALERT "Direccion lectura: %p.\n ", dpramdev.addr + filp->f_pos);
      return size;
   }  
   

   ssize_t dpramWrite(struct file *filp, const char *buff, size_t count, loff_t *offset){
      int size;
      if(access_ok(VERIFY_READ, buff, count)== 0){ 
         printk(KERN_ALERT "Fallo al acceder a la memoria de usuario\n");
         return -EFAULT;  
      }    
   // Copiamos desde la memoria de usuario a la DPRAM //
      //printk(KERN_ALERT "Tamanio: %d.\n", count);
      //printk(KERN_ALERT "offset:  %016llx(hex).\n", filp->f_pos);
      //printk(KERN_ALERT "dpramdev.addr = %p.\n", dpramdev.addr);      
      //printk(KERN_ALERT "filp->f_pos: %#llX.\n", filp->f_pos);      
      //printk(KERN_ALERT "Dir.escritura (esp.kernel): %p\n", dpramdev.addr+filp->f_pos);
      //printk(KERN_ALERT "Dir.escritura (esp.usuario): %p\n", buff);
      size=copy_from_user(dpramdev.addr+filp->f_pos, buff, count);
      return size; 
   }

/** Para implementar las funciones de carga y descarga del módulo existen dos mecanismos. 
  * Uno es implementar las funciones 'int init_module(void)' y 'void cleanup_module(void)' y el otro
  * es registrar cualquier función como función de carga o de descarga usando las macros 'module_init()' 
  * y 'module_exit()' que reciben una función como parámetro. 
  */
  
/** La funcion que se ejecutara al cargar el modulo es: dpram_init. 
  */
   module_init(dpramInit);
/** La funcion que se ejecutara al descargar el modulo es: dpram_exit. 
  */
   module_exit(dpramExit);
