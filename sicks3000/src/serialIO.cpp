#include <errno.h>
#include <fcntl.h>
#include <linux/serial.h>
#include <math.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>

#include <iostream>
#include <unistd.h>

#include "sicks3000/serialIO.h"

// Constructor que asigna un nombre por defecto.
SerialIO::SerialIO(const std::string& nombre_dispositivo) :
    ps_fd_(-1), ps_nombre_dispositivo_(nombre_dispositivo) {

        printf("SerialIO - C.\n");
    }

SerialIO::SerialIO(const SerialIO& sio) :
    ps_fd_(sio.ps_fd_), ps_nombre_dispositivo_(sio.ps_nombre_dispositivo_),
    ps_antiguo_tio_(sio.ps_antiguo_tio_),
    ps_nuevo_tio_(sio.ps_nuevo_tio_) {

        printf("SerialIO - CC.\n");

    }

SerialIO::~SerialIO() {
    printf("SerialIO - D.\n");
}

int SerialIO::abrirComunicacionSerie(const int& velocidad,
        const int& tamanio_byte,
        const SerialIO::Enum_Bandera_Bit_Stop& num_bit_stop,
        const SerialIO::Enum_Bandera_Paridad& paridad,
        const SerialIO::Enum_Bandera_Control_Flujo& control_flujo,
        int& tiempo_decimas_segundo, int& num_carac_bloq) {

    bool codigo_tasa_rxtx_valido = false;

    int codigo_tasa_rxtx = 0;
    int i_VSTART = 0;
    int i_VSTOP = 0;
    int valor_retornado = 0;

    ps_fd_ = open(ps_nombre_dispositivo_.c_str(), O_RDWR | O_NOCTTY);

    if (ps_fd_ < 0) {

        printf("Fallo abriendo el dispositivo: %s.\n",
                ps_nombre_dispositivo_.c_str());
        printf("Errno(%d): %s.\n", errno, strerror(errno));
        ps_fd_ = -1;
        return -1;
    }

    // Descarta datos recibidos en la cola de recepcion que estan esperando a ser leidos
    // y los datos escritos en la cola de transmision que estan esperando a ser transmitidos.
    if (tcflush(ps_fd_, TCIOFLUSH) == -1) {

        // En caso de error al limpiar las colas de recepcion y transmision
        // la funcion retorna de inmediato indicando la ocurrencia de un error
        // de ejecucion.
        printf(
                "No se pueden vaciar las colas de entrada y salida del puerto serie con descriptor de fichero: %d.\n",
                ps_fd_);

        // Salir ordenadamente de la funcion.
        if (close(ps_fd_) == -1) {

            printf(
                    "No se pudo cerrar con exito el fichero de dispositivo: %s (%d).\n",
                    ps_nombre_dispositivo_.c_str(), ps_fd_);
        }

        ps_fd_ = -1;
        // Indicar la ocurrencia de un error.
        return -1;
    }

    // Limpiar la estructura termios.
    bzero(&ps_antiguo_tio_, sizeof(struct termios));
    // Obtener la configuracion original del puerto serie.
    valor_retornado = tcgetattr(ps_fd_, &ps_antiguo_tio_);

    // Comprobar si la operacion se realizo con exito.
    if (valor_retornado == -1) {

        printf("Fallo en tcgetattr en el dispositivo: %s.\n",
                ps_nombre_dispositivo_.c_str());
        printf("Errno(%d): %s.\n", errno, strerror(errno));

        // Ante un error cerrar el dispositivo.
        if (close(ps_fd_) == -1) {

            printf(
                    "No se pudo cerrar con exito el fichero de dispositivo: %s (%d).\n",
                    ps_nombre_dispositivo_.c_str(), ps_fd_);
        }

        ps_fd_ = -1;
        return -1;
    }

    /* Limpiar el struct termios para recibir los nuevos parametros del puerto */
    bzero(&ps_nuevo_tio_, sizeof(struct termios));
    // c_cflag controla la velocidad (baud rate), el numero de bits por dato,
    // la paridad, los bits de parada y el control de flujo hardware.
    // El miembro c_cflag contiene dos opciones que siempre deberan de
    // estar activas, CLOCAL y CREAD. Esto nos asegura que nuestro programa no
    // se va a convertir en el "controlador" del puerto serie, permitiendole al
    // driver del puerto serie leer los bytes que no son nuestros.
    // Nunca inicialices el c_cflag(o cualquier otro) directamente; puedes usar
    // los operadores AND, OR y NOT para poner a 0 o a 1 sus bits.
    // CLOCAL  : conexion local, sin control de modem.
    // CREAD   : activa recepcion de caracteres.//
    // BAUDRATE: Fija la tasa bps. Podria tambien usar cfsetispeed y cfsetospeed.
    // CRTSCTS : control de flujo de salida por hardware (usado solo si el cable
    // tiene todas las lineas necesarias Vea sect. 7 de Serial-HOWTO)
    // CS8     : 8n1 (8bit,no paridad,1 bit de parada)
    // CLOCAL  : conexion local, sin control de modem
    // CREAD   : activa recepcion de caracteres
    // HUPCL   : If this bit is set, a modem disconnect is generated when all
    // processes that have the terminal device open have either closed the file or exited.
    ps_nuevo_tio_.c_cflag |= (CREAD | CLOCAL);
    // Configurar la velocidad.
    printf("Tasa tx_rx: %d.\n", velocidad);
    codigo_tasa_rxtx_valido
        = obtenerCodigoTasaRXTX(velocidad, codigo_tasa_rxtx);

    if (!codigo_tasa_rxtx_valido) {

        printf("Tasa rxtx a valor por defecto %d.\n", codigo_tasa_rxtx);
        // std::cout << "Baudrate code not available - setting baudrate directly" << std::endl;
        // struct serial_struct ss;
        // ioctl( ps_fd_, TIOCGSERIAL, &ss );
        // ss.flags |= ASYNC_SPD_CUST;
        // ss.custops_divisor = ss.baud_base / iNewBaudrate;
        // ioctl( ps_fd_, TIOCSSERIAL, &ss );
    }

    if ((cfsetispeed(&ps_nuevo_tio_, codigo_tasa_rxtx) != 0) || (cfsetospeed(
                    &ps_nuevo_tio_, codigo_tasa_rxtx) != 0)) {

        printf(
                "Problemas estableciendo la velocidad de la comunicacion serie.\n");
        cerrarComunicacionSerie();
        return -1;
    }

    // Formato de los datos.
    ps_nuevo_tio_.c_cflag &= ~CSIZE;

    switch (tamanio_byte) {

        case 5:
            ps_nuevo_tio_.c_cflag |= CS5;
            break;
        case 6:
            ps_nuevo_tio_.c_cflag |= CS6;
            break;
        case 7:
            ps_nuevo_tio_.c_cflag |= CS7;
            break;
        case 8:
        default:
            ps_nuevo_tio_.c_cflag |= CS8;

    }

    // |= <etiqueta> --> activa la opcion que representa la etiqueta.
    // &=~ <etiqueta> --> desactiva la opcion que representa la etiqueta.
    switch (paridad) {

        case SIO_PA_PAR://Paridad par: activar paridad, pero que no se impar.
            ps_nuevo_tio_.c_cflag |= PARENB;
            ps_nuevo_tio_.c_cflag &= ~PARODD;
            break;
        case SIO_PA_IMPAR: //Paridad impar: activar paridad y que sea par.
            ps_nuevo_tio_.c_cflag |= PARENB;
            ps_nuevo_tio_.c_cflag |= PARODD;
            break;
        case SIO_PA_NO:
        default: //Sin paridad.
            ps_nuevo_tio_.c_cflag &= ~PARENB;
            break;

    }

    switch (num_bit_stop) {

        // Usar CSTOPB implica usar 2 bits de parada.
        case SIO_BS_DOS:
            ps_nuevo_tio_.c_cflag |= CSTOPB;
            break;
            // Desactivar la opcion CSTOPB, es decir, &=~ CSTOPB, implica
            // usar 1 bit de parada.
        case SIO_BS_UNO:
        default:
            ps_nuevo_tio_.c_cflag &= ~CSTOPB;
            break;

    }

    // Control de flujo: opciones --> que no haya, que haya por hardware, que haya por software.
    // UNIX realiza el control de flujo por hardware usando la se�al de las lineas CTS (Clear To Send) y RTS (Request To Send).
    switch (control_flujo) {

        case SIO_CF_HARDWARE:
            // Activar el control del flujo por hardware.
            ps_nuevo_tio_.c_cflag |= CRTSCTS;
            // Desactivar el control de flujo por software.
            ps_nuevo_tio_.c_iflag &= ~(IXON | IXOFF | IXANY);
            break;
        case SIO_CF_XONXOFF:
            // Desactivar el control de flujo por hardware.
            ps_nuevo_tio_.c_cflag &= ~CRTSCTS;
            // Activar el control de flujo por software.
            ps_nuevo_tio_.c_iflag |= (IXON | IXOFF | IXANY);
            i_VSTART = 17;
            i_VSTOP = 19;
            break;
        case SIO_CF_NO:
        default:
            // Desactivar el control de flujo por hardware.
            // En lugar de usar la opcion &= ~CRTSCTS, se puede usar la opcion
            //  &= ~CNEW_RTSCTS; ya que CRTSCTS y CNEW_RTSCTS son sinonimos.
            ps_nuevo_tio_.c_cflag &= ~CRTSCTS;
            // Desactivar el control de flujo por software.
            ps_nuevo_tio_.c_iflag &= ~(IXON | IXOFF | IXANY);
            break;
    }

    // Choosing Processed Output
    // Processed output is selected by setting the OPOST option in the c_oflag member:
    // nuevo_tio.c_oflag |= OPOST;

    // Of all the different options, you will only probably use the ONLCR
    // option which maps newlines into CR-LF pairs. The rest of the output options
    // are primarily historic and date back to the time when line printers and terminals could
    // not keep up with the serial data stream!

    // Escogiendo 'salida en crudo'.
    ps_nuevo_tio_.c_oflag &= ~OPOST;

    // El miembro c_lflag controla el modo en que se procesan los caracteres de
    // entrada por el controlador del puerto serie. Normalmente se configura en los
    // modos canonical input o raw input.

    // Choosing Canonical Input
    // Canonical input is line-oriented. Input characters are put into a buffer
    // which can be edited interactively by the user until a CR (carriage return) or
    // LF (line feed) character is received.
    // When selecting this mode you normally select the ICANON, ECHO, and ECHOE options:
    // nuevo_tio.c_lflag |= (ICANON | ECHO | ECHOE);

    // Choosing Raw Input
    // Raw input is unprocessed. Input characters are passed through exactly as
    // they are received, when they are received. Generally you'll deselect the
    // ICANON, ECHO, ECHOE, and ISIG options when using raw input:
    // nuevo_tio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    // A Note About Input Echo
    // Never enable input echo (ECHO, ECHOE) when sending commands to
    // a MODEM or other computer that is echoing characters, as you will generate a
    // feedback loop between the two serial interfaces.
    ps_nuevo_tio_.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    // NOTA:
    // Si ICANON es usado tambien se puede usar:
    // VEOF, VEOL, VERASE, VKILL (and also VEOL2, VSTATUS and VWERASE if defined and IEXTEN is set)
    // Si ISIG es usado tambien se puede usar:
    // VINTR, VQUIT, VSUSP (and also VDSUSP if defined and IEXTEN is set)
    // Si IXON o IXOFF es usado tambien se puede usar:
    // VSTOP, VSTART
    // Timeouts are ignored in canonical input mode or when the NDELAY option is set on the file via open or fcntl
    // Si ICANON no es usado se puede usar
    // VMIN, VTIME

    // VMIN > 0, VTIME = 0.
    // VMIN fija el numero de caracteres a recibir antes de que la lectura este realizada.
    // Como VTIME es 0, el temporizador no se usa. LLAMADA BLOQUEANTE.
    // Las lecturas se bloquearan (esperara) indefinidamente a menos que NDELAY se ponga en la
    // llamada open o fcntl.
    // fcntl(fd, F_SETFL, FNDELAY) hace que la llamada a read no se bloquee y
    // devuelva un cero si no hay caracteres disponibles en el puerto.

    // Para retornar al bloqueo se debe llamar a la funcion fcntl() otra vez
    // sin el flag FNDELAY, es decir,
    // fcntl(fd, F_SETFL, 0)

    // VMIN > 0, VTIME > 0.
    // VTIME indica un temporizador entre caracteres. La lectura se realiza si:
    //    1. Se reciben VMIN caracteres (respetando la temporizacion entre caracteres).
    //    2. El tiempo entre dos caracteres excede el valor VTIME.
    // El temporizador se reinicia cada vez que se recibe un caracter y solo se hace
    // activo una vez que se ha recibido el primer caracter.

    // VMIN = 0, VTIME = 0
    // La lectura se realiza inmediatamente. Devuelve el numero de caracteres disponibles
    // en el momento, o el numero de caracteres solicitados. Se puede poner un
    // fcntl(fd, F_SETFL, FNDELAY) antes de leer para obtener el mismo comportamiento.

    // VMIN = 0, VTIME > 0
    // VTIME indica el tiempo de espera. La lectura se realizara si es leido un solo caracter
    //, o si se excede VTIME (tiempo_en_seg = VTIME * 0'1 seg). Si VTIME se excede no se
    // devuelve ningun caracter.
    //
    ps_nuevo_tio_.c_cc[VINTR] = 3; // Interrupt
    ps_nuevo_tio_.c_cc[VQUIT] = 28; // Quit
    ps_nuevo_tio_.c_cc[VERASE] = 127; // Erase
    ps_nuevo_tio_.c_cc[VKILL] = 21; // Kill-line
    ps_nuevo_tio_.c_cc[VEOF] = 4; // End-of-file

    if (tiempo_decimas_segundo < 0 || num_carac_bloq < 0) {

        printf("tiempo: %d.\n", tiempo_decimas_segundo);
        printf("num_carac_bloq: %d.\n", num_carac_bloq);

        // Usar configuracion por defecto.
        if (tiempo_decimas_segundo < 0) {

            tiempo_decimas_segundo = 0;
        }

        if (num_carac_bloq < 0) {

            num_carac_bloq = 1;
        }
    }

    ps_nuevo_tio_.c_cc[VTIME] = tiempo_decimas_segundo;
    ps_nuevo_tio_.c_cc[VMIN] = num_carac_bloq;
    ps_nuevo_tio_.c_cc[VSWTC] = 0;
    ps_nuevo_tio_.c_cc[VSTART] = i_VSTART;
    ps_nuevo_tio_.c_cc[VSTOP] = i_VSTOP;
    ps_nuevo_tio_.c_cc[VSUSP] = 26;
    ps_nuevo_tio_.c_cc[VEOL] = 0; // End-of-line
    ps_nuevo_tio_.c_cc[VREPRINT] = 18;
    ps_nuevo_tio_.c_cc[VDISCARD] = 15;
    ps_nuevo_tio_.c_cc[VWERASE] = 23;
    ps_nuevo_tio_.c_cc[VLNEXT] = 22;
    ps_nuevo_tio_.c_cc[VEOL2] = 0; // Second end-of-line

    // Guardar la nueva configuracion del puerto serie.
    valor_retornado = tcsetattr(ps_fd_, TCSANOW, &ps_nuevo_tio_);

    if (valor_retornado == -1) {

        printf("Fallo en tcsetattr del dispositivo: %s.\n",
                ps_nombre_dispositivo_.c_str());
        printf("Errno(%d): %s.\n", errno, strerror(errno));
        cerrarComunicacionSerie();
        return -1;
    }

    return 0;
}

void SerialIO::cerrarComunicacionSerie() {

    if (ps_fd_ != -1) {

        // Re-establecer la configuracion original del puerto serie.
        if (tcsetattr(ps_fd_, TCSANOW, &ps_antiguo_tio_) == -1) {

            printf(
                    "No se pudo re-establecer la configuracion original del puerto serie: %s\n",
                    ps_nombre_dispositivo_.c_str());
        }

        if (close(ps_fd_)) {

            printf(
                    "No se pudo cerrar con exito el fichero de dispositivo: %s (%d).\n",
                    ps_nombre_dispositivo_.c_str(), ps_fd_);
        }

        ps_fd_ = -1;
    }
}

int SerialIO::escribeEnPuertoSerie(const void* buffer_tx_orig, const int& num_bytes_datos) {

    int num_bytes_enviados = 0;
    // Inicialmente todos los bytes estan por escribir en el puerto serie.
    int num_bytes_restantes = num_bytes_datos;

    const char* buffer_tx = (const char*)buffer_tx_orig;

    // Escribir en el puerto serie mientras haya datos pendientes de ser escritos.
    do {

        // Escritura de bytes en el puerto serie. La funcion 'write' retorna la cantidad
        // de bytes que se han escrito en el puerto serie.
        num_bytes_enviados = write(ps_fd_, buffer_tx, num_bytes_restantes);

        // Si se produce algun error se indica con el valor de retorno -1.
        if (num_bytes_enviados < 0) {

            printf("Error escribiendo datos en puerto serie.\n");
            return -1;
        }

        // Calcular cuantos bytes quedan por escribir en el puerto serie.
        num_bytes_restantes -= num_bytes_enviados;
        // Actualizar la posicion del puntero.
        buffer_tx += num_bytes_enviados;

    } while (num_bytes_restantes > 0);

    // Acabar la funcion indicando que todos los bytes pasados como argumento de
    // la funcion han sido escritos satisfactoriamente en el puerto serie.
    return 0;
}

int SerialIO::establecerConfPSOriginal() {

    // La constante TCSANOW especifica que todos los cambios se deben establecer de
    // forma inmediata sin esperar a mandar o recibir datos.
    if (tcsetattr(ps_fd_, TCSANOW, &ps_antiguo_tio_) == -1) {

        printf(
                "YYY - No se pudo re-establecer la configuracion original del puerto serie: %s.\n",
                ps_nombre_dispositivo_.c_str());
        return -1;
    }

    return 0;
}

int SerialIO::establecerControlFlujo(const int& control_flujo) {

    int valor_retornado = 0;

    struct termios newtio;

    // Resetear la estructura termios newtio.
    bzero(&newtio, sizeof(struct termios));
    // Obtener la configuraci�n actual del puerto serie.
    valor_retornado = tcgetattr(ps_fd_, &newtio);

    // Comprobar si la funcion anterior finalizo con exito.
    if (valor_retornado == -1) {

        printf(
                "Fallo en 'establecerControlFlujo - tcgetattr' del dispositivo: %s.\n",
                ps_nombre_dispositivo_.c_str());
        printf("Errno(%d): %s.\n", errno, strerror(errno));
        return -1;
    }

    switch (control_flujo) {

        case SIO_CF_NO:
            newtio.c_cflag &= ~CRTSCTS;
            newtio.c_iflag &= ~(IXON | IXOFF | IXANY);
            newtio.c_cc[VSTART] = 0; /* Ctrl-q */
            newtio.c_cc[VSTOP] = 0; /* Ctrl-s */
            break;
        case SIO_CF_HARDWARE:
            newtio.c_cflag |= CRTSCTS;
            newtio.c_iflag &= ~(IXON | IXOFF | IXANY);
            newtio.c_cc[VSTART] = 0; /* Ctrl-q */
            newtio.c_cc[VSTOP] = 0; /* Ctrl-s */
            break;
        case SIO_CF_XONXOFF:
            newtio.c_cflag &= ~CRTSCTS;
            newtio.c_iflag |= (IXON | IXOFF);//| IXANY);
            newtio.c_cc[VSTART] = 17; /* Ctrl-q */
            newtio.c_cc[VSTOP] = 19; /* Ctrl-s */
            break;
    }

    valor_retornado = tcsetattr(ps_fd_, TCSANOW, &newtio);

    if (valor_retornado == -1) {

        printf(
                "Fallo en 'establecerControlFlujo - tcsetattr' del dispositivo. %s.\n",
                ps_nombre_dispositivo_.c_str());
        printf("Errno(%d): %s.\n", errno, strerror(errno));
        return -1;
    }

    return 0;
}

int SerialIO::establecerTipoLectura(int& tiempo_decimas_segundo, int& num_carac) {

    int valor_retornado = 0;

    struct termios newtio;

    // Resetear la estructura termios newtio.
    bzero(&newtio, sizeof(struct termios));
    // Obtener la configuraci�n actual del puerto serie.
    valor_retornado = tcgetattr(ps_fd_, &newtio);

    // Comprobar si la funcion anterior finalizo con exito.
    if (valor_retornado == -1) {

        printf(
                "Fallo en 'establecerTipoLectura - tcgetattr' del dispositivo: %s.\n",
                ps_nombre_dispositivo_.c_str());
        printf("Errno(%d): %s.\n", errno, strerror(errno));
        return -1;
    }

    if (tiempo_decimas_segundo < 0 || num_carac < 0) {

        printf("establecerTipoLectura - Valor incorrecto detectado.\n");
        printf("tiempo: %d.\n", tiempo_decimas_segundo);
        printf("num_carac: %d.\n", num_carac);

        // Usar configuracion por defecto.
        if (tiempo_decimas_segundo < 0) {

            tiempo_decimas_segundo = 0;
        }

        if (num_carac < 0) {

            num_carac = 1;
        }
    }

    newtio.c_cc[VTIME] = tiempo_decimas_segundo;
    newtio.c_cc[VMIN] = num_carac;
    valor_retornado = tcsetattr(ps_fd_, TCSANOW, &newtio);

    if (valor_retornado == -1) {

        printf(
                "Fallo en 'establecerTipoLectura - tcsetattr' del dispositivo: %s.\n",
                ps_nombre_dispositivo_.c_str());
        printf("Errno(%d): %s.\n", errno, strerror(errno));
        return -1;
    }

    return 0;
}

int SerialIO::establecerVelocidad(const int& velocidad) {

    bool codigo_tasa_rxtx_valido = false;

    int codigo_tasa_rxtx = 0;
    int valor_retornado = 0;

    struct termios newtio;

    // Resetear la estructura termios newtio.
    bzero(&newtio, sizeof(struct termios));
    // Obtener la configuracion actual del puerto serie.
    valor_retornado = tcgetattr(ps_fd_, &newtio);

    // Comprobar si la funcion anterior finalizo con exito.
    if (valor_retornado == -1) {

        printf(
                "Fallo en 'establecerVelocidad - tcgetattr' del dispositivo: %s.\n",
                ps_nombre_dispositivo_.c_str());
        printf("Errno(%d): %s.\n", errno, strerror(errno));
        return -1;
    }

    codigo_tasa_rxtx_valido
        = obtenerCodigoTasaRXTX(velocidad, codigo_tasa_rxtx);

    if (!codigo_tasa_rxtx_valido) {

        printf("Tasa_rxtx a valor por defecto: %d.\n", codigo_tasa_rxtx);
        // std::cout << "Baudrate code not available - setting baudrate directly" << std::endl;
        // struct serial_struct ss;
        // ioctl( ps_fd_, TIOCGSERIAL, &ss );
        // ss.flags |= ASYNC_SPD_CUST;
        // ss.custops_divisor = ss.baud_base / iNewBaudrate;
        // ioctl( ps_fd_, TIOCSSERIAL, &ss );
    }

    if ((cfsetispeed(&newtio, codigo_tasa_rxtx) != 0) || (cfsetospeed(&newtio,
                    codigo_tasa_rxtx) != 0)) {

        printf("Problemas estableciendo la velocidad de la comunicacion.\n");
        return -1;
    }

    valor_retornado = tcsetattr(ps_fd_, TCSANOW, &newtio);

    if (valor_retornado == -1) {
        printf(
                "Fallo en 'establecerVelocidad - tcsetattr' del dispositivo: %s.\n",
                ps_nombre_dispositivo_.c_str());
        printf("Errno(%d): %s.\n", errno, strerror(errno));
        return -1;
    }

    return 0;
}

int SerialIO::leerPuertoSerieDatosCrudos(void* const buffer_rx,
        const int& num_bytes_leer) {

    int estado = 0;
    int num_bytes_rx = 0;
    int tempo = 0;

    float t = 0;

    struct termios newtio;

    struct timeval inic;
    struct timeval fin;
    struct timeval temp;

    // Obtener la configuracion actual del puerto serie.
    // En caso de que esta informacion no se obtenga satisfactoriamente finaliza
    // la funcion 'leerPuertoSerieDatosCrudos'
    // Resetear la estructura termios newtio.
    bzero(&newtio, sizeof(struct termios));
    if (tcgetattr(ps_fd_, &newtio) != 0) {

        printf("SerialIO - leerPSDC - ps_fd_: %d.\n", ps_fd_);
        printf("Error(%d): %s.\n", errno, strerror(errno));
        // printf("EBADF: %d .\n", EBADF);
        // printf("ENOTTY: %d .\n", ENOTTY);
        return -1;
    }

    else {

        estado = newtio.c_cc[VMIN];
        tempo = newtio.c_cc[VTIME];
        // VMIN = 0 y VTIME = 0;
        // La lectura se realiza inmediatamente, devolviendo el numero de caracteres
        // disponibles en el buffer de recepcion, o el numero de caracteres
        // solicitados, si los hay.

        if ((estado == 0) && (tempo == 0)) {

            return read(ps_fd_, buffer_rx, num_bytes_leer);
        }

        else {

            // VMIN > 0 y VTIME = 0.
            if (tempo == 0) {

                // Esperar el tiempo que haga falta a que todos los bytes
                // necesarios esten disponibles en el buffer de recepcion.
                do {

                    // Consulta del numero de bytes disponibles en
                    // el buffer de recepcion.
                    num_bytes_rx = numCaracteresRecibidos();

                } while (num_bytes_rx < num_bytes_leer);

                // Leer los bytes necesarios del buffer de recepcion. La funcion read
                // retorna inmediatamente.
                return read(ps_fd_, buffer_rx, num_bytes_leer);
            }

            // VMIN = 0 y VTIME > 0
            // VTIME indica un tiempo de espera. La lectura se realizara si es leido un
            // solo byte, o si se excede VTIME (t = VTIME *0.1 s). Si VTIME se
            // excede, no se devuelve ningun byte.
            else if (estado == 0) {

                gettimeofday(&inic, NULL);

                do {

                    // Comprobar cuantos bytes se han acumulado en
                    // el buffer de recepcion.
                    num_bytes_rx = numCaracteresRecibidos();
                    // Tomar una marca de tiempo, que se usara
                    // para calcular si se ha superado el valor del temporizador
                    // de espera de recepcion.
                    gettimeofday(&fin, NULL);
                    // Calcular cuanto tiempo ha pasado desde que se ha iniciado
                    // la recepcion de bytes en el pc. De este modo
                    // se conoce si ha vencido o no el temporizador de espera.
                    temp.tv_sec = fin.tv_sec - inic.tv_sec;
                    temp.tv_usec = fin.tv_usec - inic.tv_usec;
                    // Calculo del temporizador 'propio'
                    // t=((temp.tv_usec/1000.0)+temp.tv_sec*1000.0)/100.0;
                    t = (temp.tv_usec / 100000.0) + (temp.tv_sec * 10.0);

                } while (t < tempo && num_bytes_rx == 0);

                // Se ha excecido el tiempo no se retorna ningun byte.
                if (t > tempo) {

                    return -2;
                }

                else {
                    // He recibido bytes. Con que se lea un solo byte esta parte de
                    // la funcion termina. Quiza se hayan leidos mas bytes.
                    return read(ps_fd_, buffer_rx, num_bytes_rx);
                }
            }// FIN else if( estado == 0 ){

            // Si el flujo de ejecucion llega aqui entonces se tiene
            // este caso VMIN > 0 y VTIME > 0;

            // VTIME indica el tiempo, en decimas de segundo, que debe transcurrir
            // como maximo entre la recepcion de un byte y la recepcion del consecutivo.
            // La lectura finalizara si el tiempo entre la recepcion de dos bytes
            // consecutivos excede VTIME o si reciben MIN bytes, lo que antes ocurra.
            // El temporizador se reinicia cada vez que se recibe un byte,
            // y solo se hace activo una vez que se ha recibido el primer byte.

            // Nota: La variable 'VMIN' y la variable 'numBytesLeer' son
            // independientes. Por ejemplo, puede que necesite leer en
            // total 1000 bytes, numBytesLeer = 1000, y que las lecturas
            // de buffer se realicen en bloques de 100 bytes, VMIN = 100.
            // Para leer los 1000 bytes necesarios se haran 10 lecturas de buffer,
            // cada una de ellas de 100 bytes.

            // A continuacion se hace la llamada a read, que es la funcion en la que
            // se tienen en cuenta VMIN y VTIME.
            // Que pasa si VMIN < numBytesLeer?
            // El bucle de recepcion de datos finaliza cuando se han recibido
            // 'numBytesLeer' bytes o expira un temporizador 'propio', lo que
            // ocurra primero. Si se han recibido los 'numBytesLeer' bytes
            // en el buffer de recepcion antes de que expire el temporizador 'propio', al
            // invocar a read, se leeran de este buffer 'numBytesLeer' bytes
            // porque es el numero de bytes solicitado por read y ademas
            // se leeran de inmediato.
            // Si expira el temporizador 'propio' significa que no se han recibido
            // 'numBytesLeer' bytes, se ha recibido un numero inferior. En ese caso
            // se mira al buffer para ver cuantos bytes se han recibido y read
            // solicita ese numero. De nuevo los bytes se leeran de inmediato.

            else {
                // En esta funcion se hace una peque�a trampa:
                // NO SE MIDE el tiempo entre la recepcion de dos caracteres consecutivos.
                // SI SE MIDE el tiempo que debe transcurrir
                // como maximo en la recepcion de TODOS los caracteres solicitados (tiempoTx),
                // teniendo en cuenta el tiempo maximo que puede transcurrir entre
                // la recepcion de dos caracteres consecutivos (VTIME).

                // Que ganamos con esto?
                // El TIEMPO MAXIMO necesario para leer N caracteres,
                // considerando como maximo 'm' decimas de segundo entre la recepcion de dos
                // caracteres consecutivos, es decir, VTIME = m, es:
                // tiempoTx = (N-1) * m.

                // Dejando una guarda de tiempo adicional se tiene:
                // tiempoTx = N * m.

                // Pensemos en leer la palabra 'CASA', N = 4, fijando VTIME = 1.
                // Segun lo descrito anteriormente:
                // tiempoTx (estricto) = 3 * 1 = 3 decimas de segundo.
                // tiempoTx (sobredimensionado) = 4 * 1 = 4 decimas de segundo.

                // Llega en t0 la C: (Se pone en marcha el temporizador).
                // Llega en t1 la A: t1 = t0 + (1'5 decimas seg).
                // **********************************************
                // LA LECTURA DEBERIA TERMINAR AQUI PUES m = 1. *
                // **********************************************

                // Sin embargo, tiempo_transcurrido = 1'5 < 4. La funcion lectura continua.

                // Llega en t2 la S: t2 = t1 + (0'2 decimas seg).
                //                      = t0 + (1'5 + 0'2 decimas seg).
                // tiempo_transcurrido = 1'5 + 0'2 = 1'7 < 4. La funcion lectura continua.

                // Llega en t3 la A: t3 = t2 + (1 decimas seg).
                //                      = t1 + (0'2 + 1 decimas seg).
                //                      = t0 + (1'5 + 0'2 + 1 decimas).
                // tiempo_transcurrido = 1'5 + 0'2 + 1 = 2'7 < 4.

                // Cuando han transcurrido 2'5 decimas seg todos los bytes se han recibido.
                // Como 2'5 < 4, la lectura puede realizarse, devolviendo todos los caracteres
                // que se necesitaban.

                // Globalmente se ha respetado el tiempo maximo entre recepcion de caracteres
                // consecutivos aunque localmente no fue asi como se ha visto en el aso de la
                // primera A.

                // Este truco da algo de robusted al metodo de lectura.

                tempo = tempo * num_bytes_leer;
                // Anotar en que instante inicio la lectura.
                gettimeofday(&inic, NULL);

                do {

                    // Comprobar cuantos caracteres se han acumulado en
                    // el buffer de recepcion.
                    num_bytes_rx = numCaracteresRecibidos();
                    // Tomar una marca de tiempo, que se usara
                    // para calcular si se ha superado el valor del temporizador
                    // de espera de recepcion.
                    gettimeofday(&fin, NULL);
                    // Calcular cuanto tiempo ha pasado desde que se ha iniciado
                    // la recepcion de caracteres en el pc. De este modo
                    // se conoce si ha vencido o no el temporizador de espera.
                    temp.tv_sec = fin.tv_sec - inic.tv_sec;
                    temp.tv_usec = fin.tv_usec - inic.tv_usec;
                    // Calculo del temporizador 'propio'
                    // t=((temp.tv_usec/1000.0)+temp.tv_sec*1000.0)/100.0;
                    t = (temp.tv_usec / 100000.0) + (temp.tv_sec * 10.0);
                    //printf("4.\n");

                } while (t < tempo && num_bytes_rx < num_bytes_leer);

                // La lectura se realiza si se ha rebasado el tiempo de espera de recepcion
                // o si ya estan disponibles todos los bytes solicitados en el buffer de recepcion.
                // En cualquier caso hay que retornar los caracteres disponibles en ese buffer.
                return read(ps_fd_, buffer_rx, num_bytes_rx);
            }
        }
        }
    }

    int SerialIO::numCaracteresRecibidos() {

        int num_bytes_rx = 0;
        int valor_retornado = ioctl(ps_fd_, FIONREAD, &num_bytes_rx);

        if (valor_retornado == -1) {

            return -1;
        }

        return num_bytes_rx;
    }

    // El tipo de datos speed_t es un tipo de datos unsigned int que representa la velocidad
    // de recepcion o transmision de datos en un puerto serie.
    bool SerialIO::obtenerCodigoTasaRXTX(const int& tasa_rxtx,
            int& codigo_tasa_rxtx) {

        int i = 0;
        int longitud_tabla_tasas;

        const int tabla_tasas[] = { 0, 50, 75, 110, 134, 150, 200, 300, 600, 1200,
            1800, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400,
            460800, 500000, 576000, 921600, 1000000 };
        const speed_t tabla_codigos[] = { B0, B50, B75, B110, B134, B150, B200,
            B300, B600, B1200, B1800, B2400, B4800, B9600, B19200, B38400,
            B57600, B115200, B230400, B460800, B500000, B576000, B921600,
            B1000000 };
        longitud_tabla_tasas = sizeof(tabla_tasas) / sizeof(speed_t);
        // Valor por defecto.
        codigo_tasa_rxtx = B38400;

        for (i = 0; i < longitud_tabla_tasas; i++) {

            if (tabla_tasas[i] == tasa_rxtx) {

                codigo_tasa_rxtx = tabla_codigos[i];
                return true;
            }
        }

        return false;
    }
