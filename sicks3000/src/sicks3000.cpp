#include <stdio.h>
#include <errno.h>
#include "sicks3000/sicks3000.h"

/** Alcance maximo del Sick S3000 en cm. */
const unsigned int Sicks3000::alcance_laser_cm_ = 700;
/** Numero de medidas laser que toma el modelo sick s3000 cuando abarca el angulo maximo a maxima resolucion. */
const unsigned int Sicks3000::num_tot_med_laser_ = 761;
/** Arco maximo barrido por el sick
 * 190 grados --> 3.31612557878923 rad
 */
const double Sicks3000::arco_tot_ = 3.31612557878923;
/** Resolucion angular maxima del modelo sick s3000
 *  0.25 grados --> 0.00436332312998582
 */
const double Sicks3000::res_ang_ = 0.00436332312998582;

/** Tabla que proporciona el fabricante del modelo sick s3000 para hacer el crc de los telegramas que
 * el pc envia al laser, y vicecersa.
 */
const unsigned short Sicks3000::tablaCRC_[256] = { 0x0000, 0x1021, 0x2042,
          0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7, 0x8108, 0x9129, 0xA14A, 0xB16B,
          0xC18C, 0xD1AD, 0xE1CE, 0xF1EF, 0x1231, 0x0210, 0x3273, 0x2252, 0x52B5,
          0x4294, 0x72F7, 0x62D6, 0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C,
          0xF3FF, 0xE3DE, 0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4,
          0x5485, 0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
          0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4, 0xB75B,
          0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC, 0x48C4, 0x58E5,
          0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823, 0xC9CC, 0xD9ED, 0xE98E,
          0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B, 0x5AF5, 0x4AD4, 0x7AB7, 0x6A96,
          0x1A71, 0x0A50, 0x3A33, 0x2A12, 0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79,
          0x8B58, 0xBB3B, 0xAB1A, 0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03,
          0x0C60, 0x1C41, 0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68,
          0x9D49, 0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
          0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78, 0x9188,
          0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F, 0x1080, 0x00A1,
          0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067, 0x83B9, 0x9398, 0xA3FB,
          0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E, 0x02B1, 0x1290, 0x22F3, 0x32D2,
          0x4235, 0x5214, 0x6277, 0x7256, 0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E,
          0xE54F, 0xD52C, 0xC50D, 0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447,
          0x5424, 0x4405, 0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D,
          0xD73C, 0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
          0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB, 0x5844,
          0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3, 0xCB7D, 0xDB5C,
          0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A, 0x4A75, 0x5A54, 0x6A37,
          0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92, 0xFD2E, 0xED0F, 0xDD6C, 0xCD4D,
          0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9, 0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2,
          0x2C83, 0x1CE0, 0x0CC1, 0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA,
          0x8FD9, 0x9FF8, 0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1,
          0x1EF0 };

/** Constructor.
 */
Sicks3000::Sicks3000(const unsigned int& num_bytes_cab_rx,
          const unsigned int& num_bytes_tot_rx,
          const Sicks3000::Enum_Tipo_Respuesta& tipo_r) :
     num_bytes_cab_rx_(num_bytes_cab_rx), num_bytes_tot_rx_(num_bytes_tot_rx),
               puerto_serie_("/dev/RS422-USB-SICK3000"), tipo_resp_(tipo_r) {

     printf("Sicks3000 - C.\n");

     // vec_ang_rad_.assign(num_tot_med_laser_, 0);
     vec_dist_cm_.assign(num_tot_med_laser_, 0);
     // vec_reflec_.assign(num_tot_med_laser_, 0);
     buffer_rx_.assign(num_bytes_tot_rx_, 0);
}

Sicks3000::~Sicks3000() {
     printf("Sicks3000 - D.\n");
     puerto_serie_.cerrarComunicacionSerie();
}

/** Para comunicarse con el modelo sick s3000 es necesario, 1 bit de comienzo, 1 bit de parada,
 * 8 bits de datos, y no paridad. (pagina 36 'Telegram Listing CMS').
 */

int Sicks3000::abrirComunicacionSerie(const unsigned int& velocidad) {

     int num_carac_bloq = 1;
     int tiempo_decimas_segundo = 0;

     return puerto_serie_.abrirComunicacionSerie(velocidad, 8,
               SerialIO::SIO_BS_UNO, SerialIO::SIO_PA_NO, SerialIO::SIO_CF_NO,
               tiempo_decimas_segundo, num_carac_bloq);
}

unsigned short Sicks3000::calcularCRC(const unsigned char* const ptr_datos,
          const unsigned int& num_bytes_datos) {

     unsigned short CRC_16 = 0xFFFF;

     unsigned int i = 0;

     for (i = 0; i < num_bytes_datos; i++) {
          // El operador "^" corresponde a la operacion logica "OR exclusivo".
          // Compara los bits uno a uno, si ambos son "1" o ambos son "0", el resultado es "0"
          // en caso contrario "1".
          CRC_16 = (CRC_16 << 8) ^ (tablaCRC_[(CRC_16 >> 8) ^ (0XFF
                    & ptr_datos[i])]);
     }

     return CRC_16;
}

int Sicks3000::compararBytesComienzo() {

     int indice = -1;
     unsigned int i = 0;

     for (i = 0; i < num_bytes_tot_rx_ && indice == -1; i++) {

          if ((buffer_rx_[i] == 0x00) && (buffer_rx_[i + 1] == 0x00)
                    && (buffer_rx_[i + 2] == 0x00) && (buffer_rx_[i + 3] == 0x00)
                    && (buffer_rx_[i + 4] == 0x00) && (buffer_rx_[i + 5] == 0x00)
                    && (buffer_rx_[i + 8] == 0xFF) && (buffer_rx_[i + 9] == 0X07)
                    && (buffer_rx_[i + 10] == 0x02) && (buffer_rx_[i + 11] == 0X01)
                    && (buffer_rx_[i + 12] == 0x00) && (buffer_rx_[i + 13] == 0X00)
                    && (buffer_rx_[i + 20] == 0xBB) && (buffer_rx_[i + 21] == 0XBB)
                    && (buffer_rx_[i + 22] == 0x11) && (buffer_rx_[i + 23] == 0X11)) {

               indice = i;
          }
     }

     return indice;
}

int Sicks3000::descartarDatosRXTX() {
     return puerto_serie_.descartarDatosColaRXTX();
}

bool Sicks3000::obtenerRespContinua(unsigned int& num_bytes_extra) {

     unsigned short crc_tel_rx_obt = 0;
     unsigned short crc_tel_rx_calc = 0;

     int pos_comienzo = 0;
     int valor_ret = 0;

     unsigned int dato = 0;
     unsigned int i = 0;
     unsigned int pos = 0;
     unsigned int num_bytes_resp_cont = 0;

     // unsigned int num_bytes_rx = 0;
     // unsigned int num_palabras = 0;

     // 'ptr_buffer_rx' apunta primer byte de la respuesta del laser.
     unsigned char* ptr_buffer_rx = &buffer_rx_[0];

     // Resetear el buffer de recepcion.
     buffer_rx_.assign(buffer_rx_.size(), 0);
     // Leer la respuesta del laser.
     valor_ret = puerto_serie_.leerPuertoSerieDatosCrudos((void*) ptr_buffer_rx,
               num_bytes_tot_rx_);

     // Comprobar si se pudo leer con exito el puerto serie.
     if (valor_ret == -1 || valor_ret == -2) {

          printf("Sicks3000 - ORC - valor_ret: %d.\n", valor_ret);
          printf("Fallo al leer el telegrama del puerto serie.\n");
          return false;
     }
     // Descartar datos no leidos.
     valor_ret = descartarDatosRXTX();

     // Comprobar si se pudo leer con exito el puerto serie.
     if (valor_ret == -1 || valor_ret == -2) {

          printf("Sicks3000 - ORC - valor_ret: %d.\n", valor_ret);
          printf("Fallo al leer el telegrama del puerto serie.\n");
          return false;
     }

     pos_comienzo = compararBytesComienzo();

     if (pos_comienzo == -1) {

          printf("Sicks3000 - ORC - No se encontro el comienzo del telegrama.\n");
          return false;
     }

     // En el bloque 12 y en el bloque 112, el valor de numBytesRX debe ser igual a:
     // num_bytes_tot_rx_12_ = num_bytes_cab_rep + num_bytes_datos_12 + num_bytes_crc = 6 + 1524 + 2 = 1532.
     // num_bytes_tot_rx_112_ = num_bytes_cab_rep + num_bytes_datos_112 + num_bytes_crc = 6 + 1532 (max resol.) + 2 = 1540.
     // En ninguna de las operaciones anteriores se ha tenido en cuenta los cuatro bytes a cero con los
     // que comienzan las respuesta del laser.
     // En 'num_bytes_tot_rx_' se tiene almacenado el numeto total de bytes leidos del puerto serie.
     num_bytes_resp_cont = num_bytes_tot_rx_ - num_bytes_extra;

     // Obtener el CRC del telegrama recibido.
     // Los dos ultimos bytes del telegrama enviado desde el laser al pc (laser --> pc)
     // contienen el crc del paquete.
     // Pag.36 'Telegram Listing CDS' -> Cuando se trasmiten datos, primero se transmite el byte
     // menos significativo y luego el mas significativo.
     crc_tel_rx_obt = (buffer_rx_[pos_comienzo + num_bytes_resp_cont - 1] << 8)
               | buffer_rx_[pos_comienzo + num_bytes_resp_cont - 2];
     // Calcular el CRC del telegrama obtenido. En el calculo del crc solo se deben tomar en cuenta
     // los metadatos y los datos. Es decir no se tienen en cuanta los
     // cuatro primeros bytes a '0' de la respuesta laser-->pc ni tampoco los dos ultimos bytes de dicha
     // respuesta que es el crc obtenido en la instruccion anterior.
     crc_tel_rx_calc = calcularCRC(&buffer_rx_[pos_comienzo + 4],
               num_bytes_resp_cont - 6);
     // printf("crc_tel_rx_obt: %d - crc_tel_rx_calc: %d.\n", crc_tel_rx_obt, crc_tel_rx_calc);

     // Obtener las medidas del laser.
     if (crc_tel_rx_obt == crc_tel_rx_calc) {

          pos = pos_comienzo + num_bytes_cab_rx_;

          for (i = 0; i < num_tot_med_laser_; i++) {
               // Pag.36 'Telegram Listing CDS' -> Cuando se trasmiten datos, primero se transmite el byte
               // menos significativo y luego el mas significativo.
               dato = (buffer_rx_[pos + 1] << 8) | buffer_rx_[pos];
               vec_dist_cm_[i] = dato & 0x1FFF;
               pos = pos + 2;
          }
          return true;
     }
     // printf("crc_tel_rx_obt y crc_tel_rx_calc son distintos.\n");

     // Obtener el numero de bytes utiles recibidos. Comparar este numero con el numero de bytes utiles esperados.
     // num_palabras = (buffer_rx_[pos_comienzo + 6] << 8)
     //        | buffer_rx_[pos_comienzo + 7];
     // Cada palabra esta constituida por dos bytes. De ahi que para pasar de palabras a bytes
     // haya que multiplicar por dos. El producto eficiente por potencias de dos consiste en desplazamientos
     // de bits. Multiplicar por dos, desplazar un bit a la derecha.
     // num_bytes_rx = num_palabras << 1;
     // if (num_bytes_rx != (num_bytes_resp_cont - 4)) {
     // printf("El numero de bytes esperado y el recibido no coinciden.\n");
     // return false;
     //}
     return false;
}

// Cabecera del telegrama de tipo 'fetch' que pide las lecturas del laser (bloque 12).
// const unsigned char Sicks3000::cab_bloq_12_[10] = { 0x00, 0x00, 0x45, 0x44, 0x0C, 0x00, 0x02, 0xFE, 0xFF, 0x07 };
// Cabecera del telegrama tipo 'fetch' que pide el token al laser (bloque 25).
// const unsigned char Sicks3000::cab_bloq_25_[10] = { 0x00, 0x00, 0x41, 0x44, 0x19, 0x00, 0x00, 0x05, 0XFF, 0x07 };
// Cabecera del telegrama de tipo fetch que pide las lecturas extendidas del laser (bloque 112).
// const unsigned char Sicks3000::cab_bloq_112_[10] = { 0x00, 0x00, 0x45, 0x44, 0x70, 0x00, 0x03, 0x02, 0xFF, 0x07 };

/*void Sicks3000::crearPeticion(unsigned char* ptr_buffer_tx,
 const Sicks3000::Enum_Tipo_Respuesta& id_bloque,
 const unsigned char* datos, const unsigned int& tamanio_datos) {

 unsigned char* ptr_cab = NULL;
 unsigned char* ptr_datos = NULL;

 unsigned short crc = 0;
 unsigned short crc_lsb = 0;
 unsigned short crc_msb = 0;

 // Crear un telegrama u otro dependiendo del valor de idTelegradama.
 if (id_bloque == 12) {

 ptr_cab = (unsigned char*) cab_bloq_12_;
 }

 else if (id_bloque == 25) {

 ptr_cab = (unsigned char*) cab_bloq_25_;
 }

 else if (id_bloque == 112) {

 ptr_cab = (unsigned char*) cab_bloq_112_;
 }

 memcpy(ptr_buffer_tx, ptr_cab, 10);

 // Si el telegrama a enviar al laser contiene datos, introducir estos en el bloque
 // de datos del telegrama.
 if (datos != NULL) {

 ptr_buffer_tx = ptr_buffer_tx + 10;
 ptr_datos = ptr_buffer_tx;
 // En cada telegrama que deba enviar datos al laser, los primeros seis bytes son campos
 // repetidos de la cabecera: direccion destino/fuente (2 bytes), tamanio (2 bytes),
 // flag de coordinacion (1 byte), direccion de dispositivo (1 byte).
 // A continuacion de estos seis bytes van los datos enviados al laser.
 ptr_cab = ptr_cab + 4;
 memcpy(ptr_buffer_tx, ptr_cab, 6);
 ptr_buffer_tx = ptr_buffer_tx + 6;
 memcpy(ptr_buffer_tx, datos, tamanio_datos * sizeof(char));
 ptr_buffer_tx = ptr_buffer_tx + tamanio_datos;
 // Los dos ultimos bytes del telegrama enviado al laser constituyen el CRC de este.
 // El CRC se calcula con los 6 bytes repetidos de la cabecera y los datos que se
 // envian en el telegrama.
 crc = calcularCRC(ptr_datos, 6 + tamanio_datos);
 crc_msb = (crc & 0xFF00) >> 8;
 crc_lsb = crc & 0x00FF;
 memcpy(ptr_buffer_tx, &crc_lsb, 1);
 memcpy(ptr_buffer_tx + 1, &crc_msb, 1);
 }
 }

 int Sicks3000::liberarToken() {

 int codigo_error = 0;
 int num_bytes_recibidos = 0;
 int valor_ret = 0;

 unsigned char buffer_rx[4];
 // Datos enviados del pc --> laser para que este libere el token asignado al pc.
 unsigned char datos_tx[2] = { 0X00, 0X00 };

 std::vector<unsigned char> buffer_tx(20, 0);

 // Crear el telegrama que el pc debe enviar al laser.
 crearPeticion(&buffer_tx[0], Sicks3000::RESP_25, datos_tx, 2);
 valor_ret = puerto_serie_.escribeEnPuertoSerie((void*) &buffer_tx[0], 20);

 if (valor_ret != 0) {

 printf("Error enviando telegrama de liberar token al laser.\n");
 return -1;
 }

 // Envio de respuesta laser --> pc (Recepcion de datos en el pc).
 num_bytes_recibidos = puerto_serie_.leerPuertoSerieDatosCrudos(
 (void*) buffer_rx, 4);

 if (num_bytes_recibidos < 0 || num_bytes_recibidos < 4) {

 printf("Error recibiendo respuesta de liberar token del laser.\n");
 return -1;
 }

 codigo_error = (int) (0X000000FF & buffer_rx[3]);
 return codigo_error;
 }

 bool Sicks3000::obtenerResp_12_112() {
 // Resetear el buffer de recepcion.
 buffer_rx_.assign(bufferRX.size(), 0);
 // Cuando la lectura del puerto serie se complete, 'ptrBufferRX' apuntara al
 // primer byte de la respuesta del laser.
 char* ptrBufferRX = &buffer_rx_[0];
 std::vector<unsigned char> bufferTX(10, 0);
 // Crear telegrama a enviar al laser.
 crearPeticion(&bufferTX[0], tipoResp, NULL, 0);
 // Enviar peticion al laser.
 puerto_serie_.escribeEnPuertoSerie(&bufferTX[0], 10);
 // Leer la respuesta del laser.
 int valorRetornado = puerto_serie_.leerPuertoSerieDatosCrudos(
 (void *) ptrBufferRX, numBytesTelegramaRX);
 // Comprobar si se pudo leer con exito el puerto serie.
 if (valorRetornado == -1 || valorRetornado == -2) {
 std::cout << "Fallo al leer el telegrama del puerto serie.\n"
 << std::endl;
 return false;
 }
 // El tercer byte de la respuesta del laser contiene un codigo de error. Si
 // su valor es cero, no hubo errores, en caso contrario si los hubo.
 unsigned char codigoError = bufferRX[3];
 if (codigoError != 0) {
 std::cout << "Codigo de error distinto de cero.\n" << std::endl;
 return false;
 }
 // Obtener el numero de bytes utiles recibidos. Comparar este numero con el numero de bytes utiles esperados.
 unsigned int numPalabras = (bufferRX[6] << 8) | bufferRX[7];
 // Cada palabra esta constituida por dos bytes. De ahi que para pasar de palabras a bytes
 // haya que multiplicar por dos. El producto eficiente por potencias de dos consiste en desplazamientos
 // de bits. Multiplicar por dos, desplazar un bit a la derecha.
 unsigned int numBytesRX = numPalabras << 1;
 // En el bloque 12 y en el bloque 112, el valor de numBytesRX debe ser igual al:
 // numBytesRX_12 = numBytesCabRep + numBytesDatos_12 + numBytesCRC = 6 + 1524 + 2 = 1532.
 // numBytesRX_112 = numBytesCabRep + numBytesDatos_112 + numBytesCRC = 6 + 1532 (max resol.) + 2 = 1540.
 // En ninguna de las operaciones anteriores se ha tenido en cuenta los cuatro bytes a cero con los
 // que comienzan las respuesta del laser.
 // En numBytesTelegrama se tiene almacenado el numeto total de bytes de la respuesta del laser, incluyendo
 // los 4 bytes a cero del principio. Para ajustar ambos numero es necesario hacer una resta.
 if (numBytesRX != (numBytesTelegramaRX - 4)) {
 return false;
 }
 // Obtener el CRC del telegrama recibido.
 // Los dos ultimos bytes del telegrama enviado desde el laser al pc contienen el crc del paquete.
 unsigned short crcTelegRXObt = (bufferRX[numBytesTelegramaRX - 1] << 8)
 | bufferRX[numBytesTelegramaRX - 2];
 // Calcular el CRC del telegrama obtenido. En el calculo del crc solo se deben tomar en cuenta
 // los 6 bytes de la cabecera repetida y los bytes de datos. Es decir no se tienen en cuanta los
 // cuatro primeros bytes de la respuesta laser-->pc ni tampoco los dos ultimos bytes de dicha
 // respuesta que es el crc obtenido en la instruccion anterior.
 unsigned short crcTelegRXCalc = calcularCRC(&bufferRX[4],
 numBytesTelegramaRX - 6);
 // Obtener las medidas del laser.
 if (crcTelegRXObt == crcTelegRXCalc) {
 unsigned int dato = 0;
 unsigned int numMedLaser = vecDist_CM.size();
 // Para entender esta instruccion, acude al fichero de matlab 'indiceMedidaInicial'
 unsigned int indiceMedidaInicial = ((numTotMedLaser - 1)
 - ((numMedLaser - 1) * coefIncr)) << 1;
 // El bloque 12 tiene 2 bytes de 'monitoring data' antes de los datos de las medidas
 unsigned int pos = numBytesCABRX + 2;
 if (tipoResp == RESP_112) {
 // El bloque 112 tiene 10 bytes entre 'numero de telegrama' (2 bytes), 'numero de barrido' (2 bytes) y
 // 'monitoring data (2 bytes), antes de los datos de las medidas.
 // Como ya se han sumado los 2 bytes de monitoring data, solo hace falta sumar otros 8 bytes.
 pos = pos + 8;
 }
 // El valor de 'indiceMedidaInicial' me dice a partir de que medida tengo que escoger datos del laser.
 // Lo que pasa es que cada medida ocupa dos bytes. De ahi el producto por dos. La manera
 // eficiente de hacer productos por potencia de dos es hacer desplazamientos de bits.
 // Multiplicar por dos, desplazar 1 bit a la derecha.
 pos = pos + (indiceMedidaInicial << 1);
 // Entre dos medidas consecutivas escogidas hay 'coefIncr' medidas. Cada medida contiene dos bytes.
 unsigned int offset = coefIncr << 1;

 for (unsigned int indiceMedida = 0; indiceMedida < numMedLaser; indiceMedida++) {
 // En la medida primero se transmite el byte menos significativo y despues el mas significativo.
 // Asi lo dice el manual: pag.36
 dato = (bufferRX[pos + 1] << 8) | bufferRX[pos];
 vecDist_CM[indiceMedida] = dato & 0x1FFF;
 vecAng_RAD[indiceMedida] = anguloInicial_RAD + (indiceMedida
 * incrAngular_RAD);
 vecReflec[indiceMedida] = (dato & 0x2000) >> 13;

 pos = pos + offset;
 }
 return true;
 }
 return false;

 }

 int Sicks3000::obtenerToken() {

 int num_bytes_recibidos = 0;
 int valor_ret = 0;

 unsigned int i = 0;

 // Mirar pag 65 Telegram Listing CMS. En los datos primero se debe aniadir
 // el Low Byte (0x07) y a continuacion el High Byte (0x0F).
 const unsigned char datos_tx[2] = { 0X07, 0X0F };

 unsigned char buffer_rx[4] = { 0X00, 0X00, 0X00, 0X00 };

 std::vector<unsigned char> buffer_tx(20, 0);

 // Crear telegrama a enviar al laser.
 crearPeticion(&buffer_tx[0], Sicks3000::RESP_25, datos_tx, 2);

 // Enviar el telegrama al laser.
 for (i = 0; i < 20; i++) {
 printf("%d - %#x.\n", i, buffer_tx[i]);
 }

 valor_ret = puerto_serie_.escribeEnPuertoSerie((void*) &buffer_tx[0], 20);

 if (valor_ret != 0) {

 printf("Error enviando token al laser.\n");
 return -1;
 }

 // Leer la respuesta del laser.
 num_bytes_recibidos = puerto_serie_.leerPuertoSerieDatosCrudos(
 (void*) buffer_rx, 4);

 if (num_bytes_recibidos < 0 || num_bytes_recibidos < 4) {

 printf("Error recibiendo token del laser");
 printf("num_bytes_recibidos: %d.\n", num_bytes_recibidos);
 return -1;
 }

 // Retornar el codigo de error.
 return (int) buffer_rx[3];
 }
 */
