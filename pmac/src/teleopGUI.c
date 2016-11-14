#include <gdk-pixbuf/gdk-pixbuf.h>
#include <glib-2.0/glib.h>
#include <gtk/gtk.h>
#include <GL/glut.h>
#include <GL/glu.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>
//#include <glib-2.0/glib/gthread.h>
#include <pthread.h>
#include "pmac.h"

GtkWidget *vent_teleop;
// Backing pixmap for drawing area para teleoperacion
static GdkPixmap *pixmap = NULL;
// Area de dibujo de teleoperacion: Drawing area --> da
GtkWidget *da_teleop;
int anchura_da_teleop = 400;
int anchura_da_teleop_medios = 200;
int altura_da_teleop = 400;
int altura_da_teleop_medios = 200;
int boton_apretado = 0;
// Velocidad de desplazamiento (velocidad lineal) expresada en m/seg.
int vel_desp = 0;
// Velocidad angular, expresada en rad/seg.
int vel_ang = 0;

GtkWidget *pixmapwid;
GdkFont *font;
GdkFont *font1;

int fd_dpram;


// Flag para impedir dos ventanas de teleoperacion
int lanzado_teleop = 0;
// Velocidades por defecto del motor 1 y del motor 2, expresadas en
// cuentas/mseg
char *vel_m1="I122=1";
char *vel_m2="I222=1";

// Restaura las velocidades por defecto de los motores 1 y 2.
// A continuación destruye la ventana.
 void destruirTeleoperacion(GtkWidget *widget, gpointer data){
   //comunicacion_dpram(fd,velocidad_pordefecto1,buffer_pmac_pc,&error_pmac);
   //comunicacion_dpram(fd,velocidad_pordefecto2,buffer_pmac_pc,&error_pmac);
   gtk_widget_destroy(GTK_WIDGET(vent_teleop));
   gtk_main_quit();
   }

static gint configure_event(GtkWidget *widget, GdkEventConfigure *event){

  if (pixmap){
    g_object_unref (pixmap);
  }

  pixmap = gdk_pixmap_new(widget->window, widget->allocation.width, widget->allocation.height, -1);
  gdk_draw_rectangle(pixmap, widget->style->white_gc, TRUE, 0, 0, widget->allocation.width, widget->allocation.height);

// Pintar escalas
// Eje x
  gdk_draw_line(pixmap, da_teleop->style->black_gc, 5, (anchura_da_teleop/2) - 1, 395, (anchura_da_teleop/2) - 1);
  gdk_draw_line(pixmap, da_teleop->style->black_gc, 5, (anchura_da_teleop/2)    , 395, (anchura_da_teleop/2)    );
  gdk_draw_line(pixmap, da_teleop->style->black_gc, 5, (anchura_da_teleop/2) + 1, 395, (anchura_da_teleop/2) + 1);
// Eje y
  gdk_draw_line(pixmap, da_teleop->style->black_gc, (anchura_da_teleop/2) - 1, 5, (anchura_da_teleop/2) - 1, 395);
  gdk_draw_line(pixmap, da_teleop->style->black_gc, (anchura_da_teleop/2)    , 5, (anchura_da_teleop/2)    , 395);
  gdk_draw_line(pixmap, da_teleop->style->black_gc, (anchura_da_teleop/2) + 1, 5, (anchura_da_teleop/2) + 1, 395);

// Flechas
// Flecha derecha
  gdk_draw_line(pixmap, da_teleop->style->black_gc, 390, (anchura_da_teleop/2) - 4, 398, (anchura_da_teleop/2)    );
  gdk_draw_line(pixmap, da_teleop->style->black_gc, 390, (anchura_da_teleop/2) - 4, 390, (anchura_da_teleop/2) + 4);
  gdk_draw_line(pixmap, da_teleop->style->black_gc, 390, (anchura_da_teleop/2) + 4, 398, (anchura_da_teleop/2)    );
  gdk_draw_line(pixmap, da_teleop->style->black_gc, 391, (anchura_da_teleop/2) - 3, 391, (anchura_da_teleop/2) + 3);
  gdk_draw_line(pixmap, da_teleop->style->black_gc, 391, (anchura_da_teleop/2) - 2, 397, (anchura_da_teleop/2)    );
  gdk_draw_line(pixmap, da_teleop->style->black_gc, 391, (anchura_da_teleop/2) + 2, 397, (anchura_da_teleop/2)    );
// Flecha izquierda
  gdk_draw_line(pixmap, da_teleop->style->black_gc, (anchura_da_teleop/2) - 4, 10, (anchura_da_teleop/2)    ,  2);
  gdk_draw_line(pixmap, da_teleop->style->black_gc, (anchura_da_teleop/2) - 4, 10, (anchura_da_teleop/2) + 4, 10);
  gdk_draw_line(pixmap, da_teleop->style->black_gc, (anchura_da_teleop/2) + 4, 10, (anchura_da_teleop/2)    ,  2);
  gdk_draw_line(pixmap, da_teleop->style->black_gc, (anchura_da_teleop/2) - 3,  9, (anchura_da_teleop/2) + 3,  9);
  gdk_draw_line(pixmap, da_teleop->style->black_gc, (anchura_da_teleop/2) - 2,  9, (anchura_da_teleop/2)    ,  3);
  gdk_draw_line(pixmap, da_teleop->style->black_gc, (anchura_da_teleop/2) + 2,  9, (anchura_da_teleop/2)    ,  3);
  gdk_draw_line(pixmap, da_teleop->style->black_gc, (anchura_da_teleop/2) - 2,  7, (anchura_da_teleop/2) + 2,  7);

// Cuadricula
// Ejes x
  gdk_draw_line(pixmap, da_teleop->style->black_gc, 10, (altura_da_teleop/2) - 187, 390, (altura_da_teleop/2) - 187);
  gdk_draw_line(pixmap, da_teleop->style->black_gc, 10, (altura_da_teleop/2) - 135, 390, (altura_da_teleop/2) - 135);
  gdk_draw_line(pixmap, da_teleop->style->black_gc, 10, (altura_da_teleop/2) -  68, 390, (altura_da_teleop/2) -  68);
  //gdk_draw_line(pixmap,da_teleop->style->black_gc,10,(altura_da_teleop/2)-40,390,(altura_da_teleop/2)-40);
  //gdk_draw_line(pixmap,da_teleop->style->black_gc,10,(altura_da_teleop/2)+40,390,(altura_da_teleop/2)+40);
  gdk_draw_line(pixmap, da_teleop->style->black_gc, 10,(altura_da_teleop/2) +  67, 390, (altura_da_teleop/2) +  67);
  gdk_draw_line(pixmap, da_teleop->style->black_gc, 10,(altura_da_teleop/2) + 134, 390, (altura_da_teleop/2) + 134);
  gdk_draw_line(pixmap, da_teleop->style->black_gc, 10,(altura_da_teleop/2) + 187, 390, (altura_da_teleop/2) + 187);
// Ejes y
  gdk_draw_line(pixmap, da_teleop->style->black_gc, (anchura_da_teleop/2) - 187, 10, (anchura_da_teleop/2) - 187, 390);
  gdk_draw_line(pixmap, da_teleop->style->black_gc, (anchura_da_teleop/2) - 134, 10, (anchura_da_teleop/2) - 134, 390);
  gdk_draw_line(pixmap, da_teleop->style->black_gc, (anchura_da_teleop/2) -  68, 10, (anchura_da_teleop/2) -  68, 390);
  //gdk_draw_line(pixmap,da_teleop->style->black_gc,(anchura_da_teleop/2)-40,10,(anchura_da_teleop/2)-40,390);
  //gdk_draw_line(pixmap,da_teleop->style->black_gc,(anchura_da_teleop/2)+40,10,(anchura_da_teleop/2)+40,390);
  gdk_draw_line(pixmap, da_teleop->style->black_gc, (anchura_da_teleop/2) +  67, 10, (anchura_da_teleop/2) +  67, 390);
  gdk_draw_line(pixmap, da_teleop->style->black_gc, (anchura_da_teleop/2) + 134, 10, (anchura_da_teleop/2) + 134, 390);
  gdk_draw_line(pixmap, da_teleop->style->black_gc, (anchura_da_teleop/2) + 187, 10, (anchura_da_teleop/2) + 187, 390);


  font  = (GdkFont *)gdk_font_load("-adobe-helvetica-medium-r-normal--*-100-*-*-*-*-*-*");
  font1 = (GdkFont *)gdk_font_load("-adobe-courier-medium-r-normal--*-80-*-*-*-*-*-*");

  // Se pinta la leyenda y divisiones horizontales negritas
  gdk_draw_string(pixmap,  font, da_teleop->style->black_gc,  170,  12, "V+");
  gdk_draw_string(pixmap,  font, da_teleop->style->black_gc,  165,  27, "(m/s)");

  gdk_draw_string(pixmap, font1, da_teleop->style->black_gc,  209,  12, "0.14");
  gdk_draw_line(pixmap, da_teleop->style->black_gc, 194, 12,  206,  12);
  gdk_draw_line(pixmap, da_teleop->style->black_gc, 194, 14,  206,  14);

  gdk_draw_string(pixmap, font1, da_teleop->style->black_gc,  209,  64, "0.1");
  gdk_draw_line(pixmap, da_teleop->style->black_gc, 194, 64,  206,  64);
  gdk_draw_line(pixmap, da_teleop->style->black_gc, 194, 66,  206,  66);

  gdk_draw_string(pixmap, font1, da_teleop->style->black_gc,  209,  131, "0.05");
  gdk_draw_line(pixmap, da_teleop->style->black_gc, 194, 131, 206,  131);
  gdk_draw_line(pixmap, da_teleop->style->black_gc, 194, 133, 206,  133);


  gdk_draw_string(pixmap,  font, da_teleop->style->black_gc,  170, 385, "V-");

  gdk_draw_string(pixmap, font1, da_teleop->style->black_gc,  209,  266, "-0.05");
  //gdk_draw_line(pixmap, da_teleop->style->black_gc, 194, 266, 206,  266);
  gdk_draw_string(pixmap, font1, da_teleop->style->black_gc,  209,  333, "-0.1");
  //gdk_draw_line(pixmap, da_teleop->style->black_gc, 194, 333, 206,  333);
  gdk_draw_string(pixmap, font1, da_teleop->style->black_gc,  209,  386, "-0.14");
  //gdk_draw_line(pixmap, da_teleop->style->black_gc, 194, 386, 206,  386);


  // Se pinta la leyenda
  gdk_draw_string(pixmap, font, da_teleop->style->black_gc, 20, 218, "W-");
  gdk_draw_string(pixmap, font, da_teleop->style->black_gc, 362, 218, "W+");
  gdk_draw_string(pixmap, font, da_teleop->style->black_gc, 345, 233, "(rad/s)");
  gdk_draw_string(pixmap, font1, da_teleop->style->black_gc, 10, 190, "-0.14");
  //gdk_draw_line(pixmap, da_teleop->style->black_gc, 14, 194, 14, 206);
  gdk_draw_string(pixmap, font1, da_teleop->style->black_gc, 55, 190, "-0.1");
  //gdk_draw_line(pixmap, da_teleop->style->black_gc, 67, 194, 67, 206);
  gdk_draw_string(pixmap, font1,da_teleop->style->black_gc, 123, 190, "-0.05");
  //gdk_draw_line(pixmap, da_teleop->style->black_gc, 133, 194, 133, 206);
  gdk_draw_string(pixmap, font1,da_teleop->style->black_gc, 256, 190, "0.05");
  //gdk_draw_line(pixmap, da_teleop->style->black_gc, 266, 194, 266, 206);
  gdk_draw_string(pixmap, font1,da_teleop->style->black_gc, 333, 190, "0.1");
  //gdk_draw_line(pixmap, da_teleop->style->black_gc, 333, 194, 333, 206);
  gdk_draw_string(pixmap, font1,da_teleop->style->black_gc, 376, 190, "0.14");
  gdk_draw_line(pixmap, da_teleop->style->black_gc, 386, 194, 386, 206);

  return TRUE;

  }

/* Redraw the screen from the backing pixmap */
   static gint expose_event( GtkWidget *widget, GdkEventExpose *event){
      gdk_draw_drawable(widget->window, widget->style->fg_gc[gtk_widget_get_state(widget)],
           pixmap,
           event->area.x, event->area.y,
           event->area.x, event->area.y,
           event->area.width, event->area.height);
      return FALSE;
   }


// Presionar el boton del raton.
  static gint button_press_event( GtkWidget *widget, GdkEventButton *event ){
    if (event->button == 1 && pixmap != NULL){
      boton_apretado = 1;
      vel_ang = event->x;
      vel_desp = event->y;
// La ventana tiene tamanio 'anchura_da_teleop' x 'altura_da_teleop'
// es decir -->> anchura_da_teleop_medios pixeles del (0, 0) hacia la derecha
// y otros anchura_da_teleop_medios del (0, 0) hacia la izquierda.
// Lo mismo ocurre con la altura.
// Normalizacion -->> Velocidades entre 0 y 1. Asi hago que el valor de la velocidad
// de desplazamiento y la velocidad angular sea independiente del tamaño de la ventana.
// Ojo, necesito divisón con decimales -->> necesidad de casting.
//    vel_ang  = (vel_ang - anchura_da_teleop_medios) / anchura_da_teleop_medios;
      float vdesp = ((float)vel_ang / anchura_da_teleop_medios) - 1;
//    vel_desp = (altura_da_teleop_medios - vel_desp) / altura_da_teleop_medios;
      float vang = 1 - ((float)vel_desp / altura_da_teleop_medios);
      printf("Vel_desp: %f \nVel_ang: %f \n\n", vdesp, vang);
      //teleoperacion(fd_dpram, vdesp, vang);
      }

      return TRUE;
   }


// Se libera el boton.
   static gint button_release_event( GtkWidget *widget, GdkEventButton *event ){
     if (event->button == 1 && pixmap != NULL){
       boton_apretado = 0;
// Parar al robot
       vel_ang  = 0;
       vel_desp = 0;
       //teleoperacion(fd_dpram,velocidady,velocidadx); //funciona  el "pc del robot"    -->> Esta función fran no la tiene implementada
      }
     return TRUE;
   }


// Funcion de teleoperacion: crear ventana, da_teleop, botones...
   int main( int argc, char *argv[] ){

      if( (fd_dpram = open(DPRAM, O_RDWR)) == -1 ){
        printf("No se puede abrir %s.\n", DPRAM);
        //return -1;
       }

       gtk_init(&argc, &argv);

       GtkWidget *vbox2;
       GtkWidget *button;

       vent_teleop = gtk_window_new(GTK_WINDOW_TOPLEVEL);
       gtk_widget_set_name(vent_teleop, "Teleoperacion");
       gtk_window_set_title(GTK_WINDOW(vent_teleop), "Teleoperacion");
       gtk_window_set_resizable(GTK_WINDOW(vent_teleop), FALSE);

       vbox2 = gtk_vbox_new(FALSE, 0);
       gtk_container_add(GTK_CONTAINER(vent_teleop), vbox2);
       gtk_widget_show(vbox2);

       // Create the drawing area
       da_teleop = gtk_drawing_area_new();
       gtk_widget_set_size_request(GTK_WIDGET(da_teleop), anchura_da_teleop, altura_da_teleop);
       gtk_box_pack_start(GTK_BOX(vbox2), da_teleop, TRUE, TRUE, 0);
       gtk_widget_show(da_teleop);

       button = gtk_button_new_with_label ("Cerrar");
       gtk_box_pack_start (GTK_BOX (vbox2), button, FALSE, FALSE, 0);
       gtk_widget_show (button);

       g_signal_connect(G_OBJECT(vent_teleop), "destroy", G_CALLBACK(destruirTeleoperacion), NULL);
       g_signal_connect(G_OBJECT(da_teleop), "expose_event", G_CALLBACK(expose_event), NULL);
       g_signal_connect(G_OBJECT(da_teleop), "configure_event", G_CALLBACK(configure_event), NULL);
       // Senial que detecta el un clickeo del boton.
       g_signal_connect(G_OBJECT(da_teleop), "button_press_event", G_CALLBACK(button_press_event), NULL);
       // Senial que detecta cuando dejas de presionar el boton.
       g_signal_connect(G_OBJECT(da_teleop), "button_release_event", G_CALLBACK(button_release_event), NULL);
       g_signal_connect_swapped(G_OBJECT(button), "clicked", G_CALLBACK(destruirTeleoperacion), G_OBJECT(vent_teleop));

       gtk_widget_set_events (da_teleop, GDK_EXPOSURE_MASK
                                      | GDK_LEAVE_NOTIFY_MASK
                                      | GDK_BUTTON_PRESS_MASK
                                      | GDK_BUTTON_RELEASE_MASK
                                      | GDK_POINTER_MOTION_MASK
                                      | GDK_POINTER_MOTION_HINT_MASK);

       gtk_widget_show (vent_teleop);

       gtk_main();

       return 0;

    }



