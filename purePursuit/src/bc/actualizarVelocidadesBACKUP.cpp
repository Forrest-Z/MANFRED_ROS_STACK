int actualizarVelFinalRuedas(const int & descriptorDPRAM, algoritmoPurePursuit & algPP)
{

  double radioConSignoFinalValorAbs = fabs(algPP.radioConSignoFinal);
  double velRuedaDerInicial = algPP.velRuedaDerFinal;
  double velRuedaIzqInicial = algPP.velRuedaIzqFinal;
  double vel = 0.0f;

  int flagActualizarVelocidades = 0;

  // He controlado fuera de esta funcion que el desplazamiento sea siempre hacia delante forzando que
// distanciaPP >= diametroBase, por lo tanto nunca deberia ocurrir que radioConSignoFinal < radioBase,
// aun asi se dispone de esta condicion por si fuese necesario.
  if (radioConSignoFinalValorAbs < algPP.radioBase)
  {
    cout << endl;
    cout << "\x1B[31;1m";
    cout << "Radio de giro menor que el radio de la base" << endl;
    cout << "\x1B[0m";

    algPP.radioConSignoFinal = sign(algPP.radioConSignoFinal) * algPP.radioBase;
    radioConSignoFinalValorAbs = algPP.radioBase;
  }
// Movimiento rectilineo + Giro.
  /* if(radioConSignoFinalValorAbs < algPP.radioUmbralLinRecta) */
  //else
  //{
  else if (radioConSignoFinalValorAbs > algPP.radioUmbralLinRecta)
  {
    algPP.radioConSignoFinal = sign(algPP.radioConSignoFinal) * algPP.radioUmbralLinRecta;
    radioConSignoFinalValorAbs = algPP.radioUmbralLinRecta;
  }

  double cocienteRadios = algPP.radioBase / algPP.radioConSignoFinal;

  // ===== ===== ===== ===== ===== ===== ===== ===== ===== =====

  // Se calcular una velocidad 'inicial' que tiene una dependencia lineal con el
  // radio de giro.
  vel = (algPP.velRuedaMax / algPP.radioUmbralLinRecta) * radioConSignoFinalValorAbs;

  // ===== ===== ===== ===== ===== ===== ===== ===== ===== =====

  // Giro antihorario (+)
  if (algPP.radioConSignoFinal > 0)
  {

#ifdef DEBUG

    cout << endl;
    std::cout << "___M.Rect + G.[ A ]H___\nvel: " << vel << std::endl;

#endif

    // Velocidad con un decimal.
    algPP.velRuedaDerFinal = floorf(10 * vel) / 10;
    algPP.velRuedaIzqFinal = floorf(10 * ((1 - cocienteRadios) / (1 + cocienteRadios)) * algPP.velRuedaDerFinal) / 10;
  }
  // Giro horario (-).
  else
  {

#ifdef DEBUG

    cout << endl;
    std::cout << "___M.Rect + GH___\nvel: " << vel << std::endl;

#endif

    algPP.velRuedaIzqFinal = floorf(10 * vel) / 10;
    algPP.velRuedaDerFinal = floorf(10 * ((1 + cocienteRadios) / (1 - cocienteRadios)) * algPP.velRuedaIzqFinal) / 10;
  }
  //}

  /*
   Movimiento rectilineo: radioConSignoFinalValorAbs >= algPP.radioUmbralLinRecta
   else
   {

   #ifdef DEBUG

   cout << endl;

   std::cout << "___M.Rect___" << std::endl;

   #endif

   algPP.velRuedaDerFinal = algPP.velRuedaMax;
   algPP.velRuedaIzqFinal = algPP.velRuedaMax;
   }
   */

  algPP.tiempoMediaCurvaSRuedaDer = ceilf(
      1000 * fabs(algPP.velRuedaDerFinal - velRuedaDerInicial) / algPP.acelRuedaMax);

  algPP.tiempoMediaCurvaSRuedaIzq = ceilf(
      1000 * fabs(algPP.velRuedaIzqFinal - velRuedaIzqInicial) / algPP.acelRuedaMax);

#ifdef DEBUG

  cout << endl;
  cout << "vD: " << velRuedaDerInicial << " cm/s ==> " << algPP.velRuedaDerFinal << " cm/s" << endl;
  cout << "tMCSD: " << algPP.tiempoMediaCurvaSRuedaDer << " ms " << endl;
  cout << "vI: " << velRuedaIzqInicial << " cm/s ==> " << algPP.velRuedaIzqFinal << " cm/s" << endl;
  cout << "tMCSI: " << algPP.tiempoMediaCurvaSRuedaIzq << " ms" << endl;

#endif

  int tiempoMediaCurvaSMax = algPP.tiempoCurvaSMax / 2;

  // Cuando la rueda debe mantener la velocidad constante, no hay cambio entre la orden de control
// anterior y la nueva orden de control (Vf = Vi), el tiempo de desplazamiento calculado para esa rueda
// es de 0 ms. Este resultado es debido a que se ha calculado este tiempo con la formula de
// un perfil de velocidad de tipo curva S. tiempoMed... = ceil(Vf - Vi/acelMax). Obviamente un
// tiempo de 0 ms no es correcto, lo que ocurre es que la rueda se debe mover durante to_do
// el tiempo que dura tiempoCurvaSMax a la misma velocidad.
// Deberia ser == 0 pero debido al redondeo donde debiera aparecer un 0 a veces apareace un 1.
// De ahí que se ponga un <= 1
  if (algPP.tiempoMediaCurvaSRuedaDer <= 1)
  {
    // NOTA: Si en un perfil de velocidad de tipo curva S, la velocidad inicial y final
    // son iguales, la curva S se convierte en un tramo recto.
    algPP.velRuedaDerFinal = velRuedaDerInicial;
    algPP.tiempoMediaCurvaSRuedaDer = tiempoMediaCurvaSMax;

#ifdef DEBUG

    cout << endl;
    cout << "tMCSD: 0 ms ==> " << algPP.tiempoMediaCurvaSRuedaDer << " ms " << endl;
    cout << "vDF: " << algPP.velRuedaDerFinal << " cm/s" << endl;

#endif

  }
  else if (algPP.tiempoMediaCurvaSRuedaDer > tiempoMediaCurvaSMax)
  {
    algPP.tiempoMediaCurvaSRuedaDer = tiempoMediaCurvaSMax;
    // Rectificar la velocidad de la rueda derecha. Se calcula cual es la velocidad maxima
    // que se puede conseguir usando una aceleracion acelRuedaMax y un tiempo de media
    // curva S de valor tiempoMediaCurvaSMax.
    int signoDeltaVelRuedaDer = sign(algPP.velRuedaDerFinal - velRuedaDerInicial);
    vel = velRuedaDerInicial
        + (signoDeltaVelRuedaDer * algPP.acelRuedaMax * ((double)algPP.tiempoMediaCurvaSRuedaDer / 1000.0f));

    // floorf o ceilf en funcion de si la velocidad aumenta o disminuye para no exceder la aceleracion
    // maxima permitida.
    if (signoDeltaVelRuedaDer > 0)
    {
      algPP.velRuedaDerFinal = floorf(10 * vel) / 10;
    }
    else
    {
      algPP.velRuedaDerFinal = ceilf(10 * vel) / 10;
    }

#ifdef DEBUG

    cout << endl;
    cout << "tMCSD > tMCSMAX" << endl;
    cout << "vD: " << velRuedaDerInicial << " cm/s ==> " << algPP.velRuedaDerFinal << " cm/s" << endl;

#endif

  }
  // else if(algPP.tiempoMediaCurvaSRuedaDer < 50)
  //  {
  //  tiempoMediaCurvaSRuedaDer = Tmin;
  //  adM = abs(VdF - VdI)/(tiempoMediaCurvaSRuedaDer/1000);
  // }
  if (algPP.tiempoMediaCurvaSRuedaIzq <= 1)
  {
    algPP.velRuedaIzqFinal = velRuedaIzqInicial;
    algPP.tiempoMediaCurvaSRuedaIzq = tiempoMediaCurvaSMax;

#ifdef DEBUG

    cout << endl;
    cout << "tMCSI = 0 ms ==> " << algPP.tiempoMediaCurvaSRuedaIzq << " ms " << endl;
    cout << "vIF = " << algPP.velRuedaIzqFinal << " cm/s " << endl;

#endif

  }
  else if (algPP.tiempoMediaCurvaSRuedaIzq > tiempoMediaCurvaSMax)
  {
    algPP.tiempoMediaCurvaSRuedaIzq = tiempoMediaCurvaSMax;
    int signoDeltaVelRuedaIzq = sign(algPP.velRuedaIzqFinal - velRuedaIzqInicial);
    vel = velRuedaIzqInicial
        + (signoDeltaVelRuedaIzq * algPP.acelRuedaMax * ((double)algPP.tiempoMediaCurvaSRuedaIzq / 1000.0f));

    if (signoDeltaVelRuedaIzq > 0)
    {
      algPP.velRuedaIzqFinal = floorf(10 * vel) / 10;
    }
    else
    {
      algPP.velRuedaIzqFinal = ceilf(10 * vel) / 10;
    }

#ifdef DEBUG

    cout << endl;
    cout << "tMCSI > tMCSMAX" << endl;
    cout << "vI: " << velRuedaIzqInicial << " cm/s ==> " << algPP.velRuedaIzqFinal << " cm/s" << endl;

#endif

  }
  // else if (algPP.tiempoMediaCurvaSRuedaIzq < 50)
  // {
  //algPP.tiempoMediaCurvaSRuedaIz = 50;
  //aiM = abs(ViF - ViI)/(tiempoMediaCurvaSRuedaIzq/1000);
  //}
#ifdef DEBUG
  cout << endl;
  cout << "vD: " << velRuedaDerInicial << " cm/s ==> " << algPP.velRuedaDerFinal << " cm/s" << endl;
  cout << "tMCSD: " << algPP.tiempoMediaCurvaSRuedaDer << " ms" << endl;
  cout << "vI: " << velRuedaIzqInicial << " ms ==> " << algPP.velRuedaIzqFinal << " ms" << endl;
  cout << "tMCSI: " << algPP.tiempoMediaCurvaSRuedaIzq << " ms" << endl;
#endif

#ifndef PRUEBA
  return escribirDPRAMVelocidadFinalTiempoMediaCurvaS(descriptorDPRAM, algPP);
#else
  return 0;
#endif
}
