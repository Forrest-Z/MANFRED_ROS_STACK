#ifndef _SerialIO_H_
#define _SerialIO_H_

#include <termios.h>
#include <sys/select.h>
#include <string.h>
#include <string>

// Clase que modela un puerto serie.
class SerialIO {
public:
	enum Enum_Bandera_Control_Flujo {
		SIO_CF_NO, SIO_CF_HARDWARE, SIO_CF_XONXOFF
	};
	enum Enum_Bandera_Paridad {
		SIO_PA_NO, SIO_PA_PAR, SIO_PA_IMPAR,
	};
	enum Enum_Bandera_Bit_Stop {
		SIO_BS_UNO, SIO_BS_DOS
	};

	/** Constructor.*/
	SerialIO(const std::string& nombre_dispositivo);

	SerialIO(const SerialIO& sio);

	/** Destructor.*/
	~SerialIO();

	/** Configurar la comunicacion serie e iniciarla.*/
	int abrirComunicacionSerie(const int& velocidad, const int& tamanio_byte,
			const SerialIO::Enum_Bandera_Bit_Stop& num_bit_stop,
			const SerialIO::Enum_Bandera_Paridad& paridad,
			const SerialIO::Enum_Bandera_Control_Flujo& control_flujo,
			int& tiempo_decimas_segundo, int& num_carac_bloq);

	/** Finalizar la comunicacion serie. */
	void cerrarComunicacionSerie();

	/** Funcion que descarta los datos recibidos en la cola de recepcion pero no leidos
	 * por la aplicacion y los datos escritos por la aplicacion en el buffer de transmision
	 * pero no transmitidos.
	 */
	int descartarDatosColaRXTX() {

		// TCIFLUSH: descarta datos recibidos pero no leídos.
		// TCOFLUSH: descarta datos escritos pero no transmitidos.
		return tcflush(ps_fd_, TCIOFLUSH);
	}

	/** Funcion que descarta los datos recibidos en la cola de recepcion pero no leidos
	 * por la aplicacion.
	 */
	int descartarDatosColaRX() {

		return tcflush(ps_fd_, TCIFLUSH);
	}

	/** Funcion que descarta los datos escritos por la aplicacion en la cola de transmision
	 * pero no transmitidos.
	 */
	int descartarDatosColaTX() {
		return ::tcflush(ps_fd_, TCOFLUSH);
	}

	/** Transmite todos los datos que haya disponibles en la cola de transmision del
	 * puerto serie.
	 * Llamada bloqueante.
	 */
	void enviarDatosColaTX() {
		// tcdrain() espera hasta que se haya transmitido toda la salida dirigida al objeto referido por fd.
		tcdrain(ps_fd_);
	}

	int escribeEnPuertoSerie(const void* buffer_tx_orig, const int& num_bytes_datos);

	int establecerConfPSOriginal();

	int establecerControlFlujo(const int& control_flujo);

	int establecerTipoLectura(int& tiempo_decimas_segundo, int& num_carac);

	int establecerVelocidad(const int& velocidad);

	int leerPuertoSerieDatosCrudos(void* const contenedor_rx,
			const int& num_bytes_leer);

	/** Retorna el numero de caracteres disponibles en el buffer de recepcion del puerto
	 * serie.
	 */
	int numCaracteresRecibidos();

	bool obtenerCodigoTasaRXTX(const int& tasa_rxtx, int& codigo_tasa_rxtX);

	int getPSFD() const {
		return ps_fd_;
	}

	std::string getPSNombreDispositivo() const {
		return ps_nombre_dispositivo_;
	}

private:

	int ps_fd_;

	std::string ps_nombre_dispositivo_;

	termios ps_antiguo_tio_;
	termios ps_nuevo_tio_;

};
#endif
