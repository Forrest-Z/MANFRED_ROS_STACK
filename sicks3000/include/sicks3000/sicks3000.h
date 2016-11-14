#ifndef SICKS3000_INCLUDEDEF_H_
#define SICKS3000_INCLUDEDEF_H_
//-----------------------------------------------

#include <string>
#include <vector>
#include <iostream>
#include <math.h>

#include "sicks3000/serialIO.h"

/** 
 * Driver class for the laser scanner SICK S3000 Professional.
 * This driver only supports use with 500KBaud in CONTINUOS MODE.
 * 
 * Message format in page 46 of the manual
 * 'Telegram listing CMS S3000/S300 Professional CMS'
 *	   
 * | 00 00 00 00 |   4 bytes, reply header
 *
 *	Now starts the actual telegram: 
 *
 * | 00 00 |         2 bytes, data block number, 0x00 for data output.
 * | xx xx |         2 bytes, size of data telegram (should be dec 1544 -> 0x0608).
 * | FF 07 |         2 bytes, co-ordination flag and device code.
 * | 02 01 |         2 bytes, protocol version number.
 * | 0x 00 |         2 bytes, status normal: 00 00 (0x0000), lockout: 01 00 (0x0001).
 * | xx xx xx xx |   4 bytes, scan number.
 * | xx xx |         2 bytes, telegram number.
 * | BB BB |         2 bytes, the following block measurement data with the BBBB flag is output if 'measurement data
 output' and 'not alternating output' is configured, or if 'alternating output' is configured and
 the telegram number is 'even'. We have configured 'measurement data
 output' and 'not alternating output'.
 * | 11 11 |         2 bytes, ID for measurement data from angular range 1. Only one range of 190 degrees is configured.
 *	   ...           (761 * 2 = 1522) bytes, measured values from angular range 1.
 * | xx xx |         2 bytes, CRC.
 *	   
 *
 * In our CONTINUOUS MODE configuration:
 *
 *	   Reply-Header:    4 bytes, byte  0 to 3.
 *	   Telegram
 *	   	  Metadata:    20 bytes, byte    4 to    23.
 *	   	  Data:      1522 bytes, byte   24 to  1545. (761 med * 2 bytes/med = (190/0.25 + 1) med * 2 bytes/med)
 *	   	  CRC:    	    2 bytes, byte 1546 and 1547.

 *	   Total length in buffer is 1548 bytes.
 */

class Sicks3000 {

public:

	enum Enum_Tipo_Respuesta {
		RESP_12 = 12, RESP_25 = 25, RESP_112 = 112, RESP_CONT = 10000
	};

	Sicks3000(const unsigned int& num_bytes_cab_rx,
			const unsigned int& num_bytes_tot_rx,
			const Sicks3000::Enum_Tipo_Respuesta& tipo_r);

	~Sicks3000();

	int abrirComunicacionSerie(const unsigned int& velocidad);

	int compararBytesComienzo();

	bool obtenerRespContinua(unsigned int& num_bytes_extra);

	unsigned int getAlcanceLaserCM() {

		return alcance_laser_cm_;
	}

	unsigned int getNumTotMedLaser() const {
		return num_tot_med_laser_;
	}

	double getArcoTot() const {

		return arco_tot_;
	}

	double getResAng() const {

		return res_ang_;
	}

	std::vector<unsigned int> getVecDistCM() const {
		return vec_dist_cm_;
	}

	std::string getNombrePuertoSerie() const {
		return puerto_serie_.getPSNombreDispositivo();
	}

private:

	// Constantes
	static const unsigned int alcance_laser_cm_;
	static const unsigned int num_tot_med_laser_;

	static const double arco_tot_;
	static const double res_ang_;

	static const unsigned short tablaCRC_[256];

	// Variables
	unsigned short crc_rx_;unsigned short estado_barrido_rx_;

	//unsigned int coef_incr_;
	unsigned int marca_temporal_rx_;unsigned int num_barrido_rx_;unsigned int
			num_bytes_cab_rx_;unsigned int num_bytes_tot_rx_;
	//unsigned int num_med_laser_;
	unsigned int num_telegrama_rx_;

	SerialIO puerto_serie_;

	Enum_Tipo_Respuesta tipo_resp_;

	std::vector<unsigned char> buffer_rx_;

	std::vector<unsigned int> vec_dist_cm_;

	unsigned short calcularCRC(const unsigned char* const ptr_datos,
			const unsigned int& num_bytes_datos);

	int descartarDatosRXTX();

};
#endif

// double angulo_inicial_rad_;
// double angulo_final_rad_;
// double incr_angular_rad_;
// std::vector<int> vec_reflec_;
// std::vector<double> vec_ang_rad_;


// Cabecera del telegrama de tipo 'fetch' que pide las lecturas del laser (bloque 12).
// static const unsigned char cab_bloq_12_[10];
// static const unsigned char cab_bloq_25_[10];
// Cabecera del telegrama de tipo fetch que pide las lecturas extendidas del laser (bloque 112).
// static const unsigned char cab_bloq_112_[10];

// std::vector<int> getVecReflec() const {
//	 return vec_reflec_;
// }

// std::vector<double> getVecAngRad() const {
// 	 return vec_ang_rad_;
// }

// void crearPeticion(unsigned char* ptr_buffer_tx, const Sicks3000::Enum_Tipo_Respuesta& id_bloque, const unsigned char* datos, const unsigned int& tamanio_datos);
// int liberarToken();
// bool obtenerResp_12_112();
// int obtenerToken();

