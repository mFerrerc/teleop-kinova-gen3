#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <memory>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/ObjectColor.h>
#include <moveit_msgs/PlanningSceneWorld.h>
#include <moveit/collision_detection/collision_tools.h>
#include <moveit/robot_model/robot_model.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Point.h>
#include <shape_msgs/SolidPrimitive.h>
#include <geometry_msgs/Pose.h>
#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/LinkStates.h>

#include <iostream>
#include <iomanip>
#include <algorithm>
#include <cmath>
#include <map>
#include <string>
#include <vector>
#include <unordered_set>

/**
 * @file multiple_objects_deepest_contacts.cpp
 * @brief Nodo de ROS que crea y gestiona una escena de planificación en MoveIt!, 
 *        publicando objetos de colisión, detectando colisiones (sus profundidades) 
 *        y actualizando sus posiciones en tiempo real con datos de Gazebo.
 *        También maneja la publicación de marcadores en RViz y la lógica de "attach" 
 *        para manipulación de objetos con la pinza.
 */

// Alias para un map con información de contactos
using MyContactMap = std::map<std::pair<std::string, std::string>,
                              std::vector<collision_detection::Contact>>;

// Ajustar a la configuración del robot y parámetros.
static const std::string ROBOT_DESCRIPTION_PARAM = "my_gen3/robot_description";  
static const std::string PLANNING_SCENE_TOPIC    = "/my_gen3/planning_scene";
static const std::string COLLISION_OBJECT_TOPIC  = "/collision_object";
static const std::string REFERENCE_FRAME         = "base_link";

// ---------------------------------------------------------------------------------
// Funciones helper para crear mensajes de objetos de colisión (box, sphere, cylinder)
// ---------------------------------------------------------------------------------

/**
 * @brief Crea un CollisionObject con forma de caja (box).
 * @param id Identificador único del objeto.
 * @param sizeX, sizeY, sizeZ Dimensiones de la caja.
 * @param px, py, pz Posición en el marco de referencia.
 * @param qx, qy, qz, qw Orientación en forma de cuaternión.
 * @return Objeto de colisión con la geometría y pose asignadas.
 */
moveit_msgs::CollisionObject makeBox(const std::string& id,
                                     double sizeX, double sizeY, double sizeZ,
                                     double px, double py, double pz,
                                     double qx=0.0, double qy=0.0, double qz=0.0, double qw=1.0)
{
  moveit_msgs::CollisionObject obj;
  obj.id = id;
  obj.header.frame_id = REFERENCE_FRAME;

  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions = {sizeX, sizeY, sizeZ};

  geometry_msgs::Pose pose;
  pose.position.x = px;
  pose.position.y = py;
  pose.position.z = pz;
  pose.orientation.x = qx;
  pose.orientation.y = qy;
  pose.orientation.z = qz;
  pose.orientation.w = qw;

  obj.primitives.push_back(primitive);
  obj.primitive_poses.push_back(pose);
  obj.operation = obj.ADD;
  return obj;
}

/**
 * @brief Crea un CollisionObject con forma de esfera (sphere).
 * @param id Identificador único del objeto.
 * @param radius Radio de la esfera.
 * @param px, py, pz Posición en el marco de referencia.
 * @return Objeto de colisión con la geometría y pose asignadas.
 */
moveit_msgs::CollisionObject makeSphere(const std::string& id,
                                        double radius,
                                        double px, double py, double pz)
{
  moveit_msgs::CollisionObject obj;
  obj.id = id;
  obj.header.frame_id = REFERENCE_FRAME;

  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.SPHERE;
  primitive.dimensions.resize(1);
  primitive.dimensions[0] = radius;

  geometry_msgs::Pose pose;
  pose.orientation.w = 1.0;
  pose.position.x = px;
  pose.position.y = py;
  pose.position.z = pz;

  obj.primitives.push_back(primitive);
  obj.primitive_poses.push_back(pose);
  obj.operation = obj.ADD;
  return obj;
}

/**
 * @brief Crea un CollisionObject con forma de cilindro (cylinder).
 * @param id Identificador único del objeto.
 * @param length Longitud o altura del cilindro.
 * @param radius Radio del cilindro.
 * @param px, py, pz Posición en el marco de referencia.
 * @return Objeto de colisión con la geometría y pose asignadas.
 */
moveit_msgs::CollisionObject makeCylinder(const std::string& id,
                                          double length, double radius,
                                          double px, double py, double pz)
{
  moveit_msgs::CollisionObject obj;
  obj.id = id;
  obj.header.frame_id = REFERENCE_FRAME;

  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.CYLINDER;
  primitive.dimensions.resize(2);
  // dimensions[0] => longitud, dimensions[1] => radio
  primitive.dimensions[0] = length;
  primitive.dimensions[1] = radius;

  geometry_msgs::Pose pose;
  pose.orientation.w = 1.0;
  pose.position.x = px;
  pose.position.y = py;
  pose.position.z = pz;

  obj.primitives.push_back(primitive);
  obj.primitive_poses.push_back(pose);
  obj.operation = obj.ADD;
  return obj;
}

// ---------------------------------------------------------------------------------
// Publicador global de marcadores de colisión y estructura global para almacenarlos
// ---------------------------------------------------------------------------------
ros::Publisher marker_array_publisher;
visualization_msgs::MarkerArray g_collision_points;

/**
 * @brief Publica los marcadores de colisión (puntos de contacto) en RViz.
 *        Primero elimina los marcadores anteriores y luego publica los nuevos.
 * @param markers Conjunto de marcadores a publicar.
 */
void publishMarkers(const visualization_msgs::MarkerArray& markers)
{
  // Borrar marcadores anteriores publicados
  if (!g_collision_points.markers.empty())
  {
    for (auto& marker : g_collision_points.markers)
      marker.action = visualization_msgs::Marker::DELETE;
    marker_array_publisher.publish(g_collision_points);
  }

  // Publicar nuevos marcadores
  visualization_msgs::MarkerArray temp = markers;
  g_collision_points.markers.swap(temp.markers);
  if (!g_collision_points.markers.empty())
    marker_array_publisher.publish(g_collision_points);
}

/**
 * @brief Verifica si una cadena comienza con un prefijo dado.
 * @param s Cadena a evaluar.
 * @param prefix Prefijo a comprobar.
 * @return true si s inicia con prefix, false en caso contrario.
 */
bool startsWith(const std::string& s, const std::string& prefix)
{
  return (s.rfind(prefix, 0) == 0);
}

/**
 * @brief Redondea un valor double a 3 decimales.
 * @param val Valor a redondear.
 * @return Valor redondeado a 3 decimales.
 */
double round3(double val)
{
  return std::round(val * 1000.0) / 1000.0;
}

// ---------------------------------------------------------------------------------
// Variables y callbacks para el manejo del "attach" y la detección de estado de pinza
// ---------------------------------------------------------------------------------
bool attach_state = false;       ///< Estado actual de "attach" leído de /estado_attach
bool attach_state_prev = false;  ///< Estado previo, para detectar flancos
double z_prev = 0.0;             ///< Valor z previo cuando se pasa a "attached"
std::string attached_object_name = ""; ///< Nombre del objeto actualmente adherido, si existe

/**
 * @brief Callback para leer el estado de "attach" desde /estado_attach.
 * @param msg Mensaje que contiene un bool con el estado de la sujeción.
 */
void attachCallback(const std_msgs::Bool::ConstPtr& msg)
{
  attach_state = msg->data;
}

/**
 * @brief Callback que recibe el nombre del objeto adherido desde /obj_attached.
 * @param msg Mensaje de tipo String con el nombre del objeto en pinza.
 */
void objAttachedCallback(const std_msgs::String::ConstPtr& msg)
{
  attached_object_name = msg->data;
}

// ---------------------------------------------------------------------------------
// Manejo de posiciones de objetos en Gazebo: model_states y link_states
// ---------------------------------------------------------------------------------
std::map<std::string, geometry_msgs::Pose> objects_positions; ///< Posiciones globales de ciertos objetos
std::vector<std::string> target_objects = {"unit_sphere", "unit_cylinder", "box_0", "box_1"};

/**
 * @brief Callback para /gazebo/model_states, actualiza la pose global de objetos de interés.
 * @param msg Contiene los nombres y poses de todos los modelos en Gazebo.
 */
void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    for (const auto& target_object : target_objects)
    {
        auto it = std::find(msg->name.begin(), msg->name.end(), target_object);
        if (it != msg->name.end())
        {
            size_t index = std::distance(msg->name.begin(), it);
            objects_positions[target_object] = msg->pose[index];
        }
    }
}

geometry_msgs::Pose bracelet_pose;        ///< Pose de un enlace específico (ej. bracelet_link)
bool bracelet_pose_received = false;      ///< Indica si ya se recibió la pose de dicho enlace

/**
 * @brief Callback para /gazebo/link_states, busca la pose de un enlace particular del robot.
 * @param msg Mensaje con todas las poses de los enlaces en Gazebo.
 */
void linkStatesCallback(const gazebo_msgs::LinkStates::ConstPtr &msg)
{
  // Nombre del enlace en Gazebo.
  static const std::string LINK_NAME = "my_gen3::bracelet_link";

  for (size_t i = 0; i < msg->name.size(); i++)
  {
    if (msg->name[i] == LINK_NAME)
    {
      bracelet_pose = msg->pose[i];
      bracelet_pose_received = true;
      break;
    }
  }
}

// ---------------------------------------------------------------------------------
// main()
// ---------------------------------------------------------------------------------
int main(int argc, char** argv)
{
  ros::init(argc, argv, "multiple_objects_deepest_contacts");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(2);  // Spinner con 2 hilos para manejar varios callbacks
  spinner.start();

  // Publicador de MarkerArray para mostrar contactos de colisión
  marker_array_publisher = nh.advertise<visualization_msgs::MarkerArray>("collision_points", 100);

  // Varios publicadores para comunicar información de contactos y posición
  ros::Publisher num_contactos_pub    = nh.advertise<std_msgs::Int32>("/num_contactos", 1);
  ros::Publisher media_contactos_pub  = nh.advertise<geometry_msgs::Point>("/media_contactos", 1);
  ros::Publisher altura_objeto_pub    = nh.advertise<std_msgs::Bool>("/altura_objeto", 10);
  ros::Publisher posx_pub            = nh.advertise<std_msgs::Float64>("/posx", 10);
  ros::Publisher posy_pub            = nh.advertise<std_msgs::Float64>("/posy", 10);
  ros::Publisher posz_pub            = nh.advertise<std_msgs::Float64>("/posz", 10);
  ros::Publisher planning_scene_diff_pub =
        nh.advertise<moveit_msgs::PlanningScene>(PLANNING_SCENE_TOPIC, 1);
  ros::Publisher estado_attach_pub = nh.advertise<std_msgs::Bool>("/estado_attach", 100);

  // Suscriptores para el estado de attach y objetos, y para la información de Gazebo
  ros::Subscriber attach_sub = nh.subscribe("/estado_attach", 100, attachCallback);
  ros::Subscriber obj_attached_sub = nh.subscribe("/obj_attached", 100, objAttachedCallback);
  ros::Subscriber model_states_sub = nh.subscribe<gazebo_msgs::ModelStates>(
    "/gazebo/model_states", 1, modelStatesCallback);

  ros::Subscriber link_states_sub = nh.subscribe("/gazebo/link_states", 1, linkStatesCallback);

  // Publicador para la posición del "esclavo" (o enlace referenciado)
  ros::Publisher slave_position_pub =
      nh.advertise<geometry_msgs::Vector3>("/posicion_esclavo", 10);

  // Publicador para un estado de la pinza (abierta/cerrada)
  ros::Publisher estado_pinza_pub = nh.advertise<std_msgs::String>("/estado_pinza", 10);

  // Inicializamos la pinza como "abierta"
  std_msgs::String pinza_msg;
  pinza_msg.data = "abierta";
  estado_pinza_pub.publish(pinza_msg);

  // Configurar el buffer y listener de TF para transformaciones
  auto tf_buffer   = std::make_shared<tf2_ros::Buffer>(ros::Duration(10));
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  double posz = 0.0;
  double posy = 0.0;
  double posx = 0.0;

  // Crear PlanningSceneMonitor para gestionar la escena de MoveIt!
  auto psm = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(ROBOT_DESCRIPTION_PARAM, tf_buffer);
  if (!psm->getPlanningScene())
  {
    std::cout << "ERROR: No se pudo crear la escena de planificación desde "
              << ROBOT_DESCRIPTION_PARAM << std::endl;
    return 1;
  }

  // Iniciar monitores de la escena
  psm->startSceneMonitor(PLANNING_SCENE_TOPIC);
  psm->startWorldGeometryMonitor();
  psm->startStateMonitor("/my_gen3/joint_states");
  ros::Duration(2.0).sleep();

  // Publicador de CollisionObject para comunicar objetos
  ros::Publisher collision_object_publisher =
      nh.advertise<moveit_msgs::CollisionObject>(COLLISION_OBJECT_TOPIC, 10, true);
  ros::Duration(1.0).sleep();

  // ---------------------------------------------------------------------------------
  // (1) Creación y publicación de objetos en la escena: mesas, cajas, esfera y cilindro
  // ---------------------------------------------------------------------------------
  double qz_90 = 0.70710678;  // Cuaternión para rotación de 90°
  double qw_90 = 0.70710678;

  // Cajas grandes (ej. superficies de trabajo)
  auto box1 = makeBox("Mesa_trabajo_0", 1.2, 0.4, 0.3, 0.18384, 0.4617, 0.149991);
  collision_object_publisher.publish(box1);

  auto box2 = makeBox("Mesa_trabajo_1", 1.3, 0.4, 0.3,
                      0.882573, 0.007374, 0.150529,
                      0.0, 0.0, qz_90, qw_90);
  collision_object_publisher.publish(box2);

  auto box3 = makeBox("Mesa_trabajo_2", 1.3, 0.4, 0.3,
                      -0.486012, 0.010008, 0.150529,
                      0.0, 0.0, qz_90, qw_90);
  collision_object_publisher.publish(box3);

  auto box4 = makeBox("Mesa_trabajo_3", 1.2, 0.4, 0.3,
                      0.163807, -0.440545, 0.15);
  collision_object_publisher.publish(box4);

  // Cajas pequeñas (ejemplo de objetos manipulables)
  auto box6 = makeBox("box_0", 0.05, 0.05, 0.05, 0.366319, 0.367316, 0.324995);
  collision_object_publisher.publish(box6);

  auto box7 = makeBox("box_1", 0.05, 0.05, 0.05, 0.581241, 0.286861, 0.324997);
  collision_object_publisher.publish(box7);

  // Esfera
  auto sphere1 = makeSphere("unit_sphere", 0.035, 0.244203, 0.531634, 0.334993);
  collision_object_publisher.publish(sphere1);

  // Cilindro
  auto cylinder1 = makeCylinder("unit_cylinder", 0.1, 0.03, 0.462930, 0.585810, 0.349988);
  collision_object_publisher.publish(cylinder1);

  ros::Duration(1.0).sleep();

  // ---------------------------------------------------------------------------------
  // (2) Permitir colisiones en la ACM y asignar colores a cada objeto
  // ---------------------------------------------------------------------------------
  {
    planning_scene_monitor::LockedPlanningSceneRW scene(psm);
    if (!scene)
    {
      std::cout << "No se pudo bloquear la escena para escritura." << std::endl;
      return 1;
    }

    collision_detection::AllowedCollisionMatrix &acm = scene->getAllowedCollisionMatrixNonConst();
    const auto &robot_model = scene->getRobotModel();
    const auto &all_links   = robot_model->getLinkModelNames();

    // Lista de todos los IDs de objetos creados
    std::vector<std::string> all_obj_ids = {
      "Mesa_trabajo_0",
      "Mesa_trabajo_1",
      "Mesa_trabajo_2",
      "Mesa_trabajo_3",
      "box_0",
      "box_1",
      "unit_sphere",
      "unit_cylinder"
    };

    // (a) Permitir colisiones entre los objetos y todos los enlaces del robot
    for (const auto &link_name : all_links)
    {
      for (const auto &id : all_obj_ids)
      {
        acm.setEntry(id, link_name, true);
      }
    }

    // (b) Obtener la escena en un mensaje de tipo PlanningScene
    moveit_msgs::PlanningScene ps_msg;
    scene->getPlanningSceneMsg(ps_msg);
    ps_msg.is_diff = true;

    // (c) Asignar colores a cada objeto
    for (const auto &id : all_obj_ids)
    {
      moveit_msgs::ObjectColor color_msg;
      color_msg.id = id;

      if (id == "Mesa_trabajo_0" ||
          id == "Mesa_trabajo_1" ||
          id == "Mesa_trabajo_2" ||
          id == "Mesa_trabajo_3")
      {
        // Cajas grandes => gris
        color_msg.color.r = 0.5f;
        color_msg.color.g = 0.5f;
        color_msg.color.b = 0.5f;
        color_msg.color.a = 1.0f;
      }
      else if (id == "box_0" || id == "box_1")
      {
        // Cajas pequeñas => azul
        color_msg.color.r = 0.0f;
        color_msg.color.g = 0.0f;
        color_msg.color.b = 1.0f;
        color_msg.color.a = 1.0f;
      }
      else if (id == "unit_sphere")
      {
        // Esfera => verde
        color_msg.color.r = 0.0f;
        color_msg.color.g = 1.0f;
        color_msg.color.b = 0.0f;
        color_msg.color.a = 1.0f;
      }
      else if (id == "unit_cylinder")
      {
        // Cilindro => rojo
        color_msg.color.r = 1.0f;
        color_msg.color.g = 0.0f;
        color_msg.color.b = 0.0f;
        color_msg.color.a = 1.0f;
      }
      else
      {
        // Si en un futuro se añaden más objetos
        continue;
      }

      ps_msg.object_colors.push_back(color_msg);
    }

    // (d) Publicar la escena con colores y colisiones permitidas
    planning_scene_diff_pub.publish(ps_msg);
    std::cout << "Escena principal: colisiones permitidas y colores asignados." << std::endl;

    // Se puede almacenar la posición del enlace (bracelet_pose) si se recibió
    if (bracelet_pose_received)
    {
      posz = bracelet_pose.position.z;
      posy = bracelet_pose.position.y;
      posx = bracelet_pose.position.x;
    }
  }

  // ---------------------------------------------------------------------------------
  // (3) Bucle principal: detección de colisiones y publicación de resultados
  // ---------------------------------------------------------------------------------
  bool last_collision_state = false;
  bool first_loop = true;  // Para gestionar mensajes al pasar la primera vez sin colisión

  geometry_msgs::Point last_media;  // Última media de contactos con los dedos
  last_media.x = 0.0;
  last_media.y = 0.0;
  last_media.z = 0.0;

  // Lista de links asociados a los dedos de la pinza
  std::vector<std::string> finger_objs = {
    "left_inner_finger", "left_inner_finger_pad", "left_inner_knuckle",
    "left_outer_finger", "left_outer_knuckle",
    "right_inner_finger","right_inner_finger_pad","right_inner_knuckle",
    "right_outer_finger","right_outer_knuckle"
  };

  ros::Rate rate(10);  // Frecuencia de 10 Hz
  while (ros::ok())
  {
    // Actualizar la escena con el estado actual
    psm->updateSceneWithCurrentState();

    // Bloquea la escena para lectura/escritura
    planning_scene_monitor::LockedPlanningSceneRW scene(psm);
    if (!scene)
    {
      ros::spinOnce();
      rate.sleep();
      continue;
    }

    // Crear una copia 'diff' para la detección de colisiones
    auto scene_for_visualization = scene->diff();
    scene_for_visualization->setCurrentState(scene->getCurrentState());

    // Usar collision_detection::Bullet
    scene_for_visualization->setActiveCollisionDetector(
        collision_detection::CollisionDetectorAllocatorBullet::create(), true);

    // NO permitir colisiones (para que se reporten)
    collision_detection::AllowedCollisionMatrix &acm_copy =
        scene_for_visualization->getAllowedCollisionMatrixNonConst();
    const auto &robot_model = scene_for_visualization->getRobotModel();
    const auto &all_links   = robot_model->getLinkModelNames();

    std::vector<std::string> all_obj_ids = {
      "Mesa_trabajo_0", "Mesa_trabajo_1", "Mesa_trabajo_2", "Mesa_trabajo_3",
      "box_0", "box_1", "unit_sphere", "unit_cylinder"
    };

    // Configurar colisiones "no permitidas" para que se reporten
    for (const auto &link_name : all_links)
    {
      for (const auto &id : all_obj_ids)
      {
        acm_copy.setEntry(id, link_name, false);
      }
    }

    // Solicitar un reporte de colisiones detallado
    collision_detection::CollisionRequest req;
    collision_detection::CollisionResult res;
    req.contacts = true;
    req.max_contacts = 100;            // Máximo de contactos globales
    req.max_contacts_per_pair = 10;   // Máximo de contactos por par

    // Verificar colisiones
    scene_for_visualization->checkCollision(req, res);

    // Detectar si hay colisión
    bool current_collision_state = (res.collision && !res.contacts.empty());

    // Publicar el número total de contactos
    std_msgs::Int32 msg_num_contactos;
    msg_num_contactos.data = current_collision_state ? res.contact_count : 0;

    // ---------------------------------------------------------------------------------
    // Manejo de flancos en /estado_attach y actualización de pose
    // ---------------------------------------------------------------------------------
    if (bracelet_pose_received)
    {
      posz = bracelet_pose.position.z;
      posy = bracelet_pose.position.y;
      posx = bracelet_pose.position.x;

      // Publicar la posición del "esclavo"
      geometry_msgs::Vector3 slave_pose_msg;
      slave_pose_msg.x = posx;
      slave_pose_msg.y = posy;
      slave_pose_msg.z = posz;
      slave_position_pub.publish(slave_pose_msg);
    }

    if (attach_state && !attach_state_prev)
    {
      // Flanco ascendente => Objeto Attached
      if (bracelet_pose_received)
      {
        z_prev = bracelet_pose.position.z;
      }
      std::cout << "\nObjeto Attached" << std::endl;
    }
    else if (!attach_state && attach_state_prev)
    {
      // Flanco descendente => Objeto Detached
      std::cout << "\nObjeto Detached" << std::endl;
    }

    // ---------------------------------------------------------------------------------
    // Si se está "attach" => comprobar si se eleva el objeto
    // ---------------------------------------------------------------------------------
    if (attach_state)
    {
      bool is_greater = (posz > z_prev + 0.0009);

      // Publicar si la altura del objeto es mayor que la z previa
      std_msgs::Bool bool_msg;
      bool_msg.data = is_greater;
      altura_objeto_pub.publish(bool_msg);

      // Publicar posición en z, y, x
      std_msgs::Float64 msg_z;  msg_z.data = posz;  posz_pub.publish(msg_z);
      std_msgs::Float64 msg_y;  msg_y.data = posy;  posy_pub.publish(msg_y);
      std_msgs::Float64 msg_x;  msg_x.data = posx;  posx_pub.publish(msg_x);

      // Publicar también el estado de attach
      bool_msg.data = attach_state;
      estado_attach_pub.publish(bool_msg);

      // Estado de la pinza = "cerrada"
      std_msgs::String pinza_msg;
      pinza_msg.data = "cerrada";
      estado_pinza_pub.publish(pinza_msg);
    }
    else
    {
      // No está "attached", publicamos false en /altura_objeto (opcional)
      std_msgs::Bool bool_msg;
      bool_msg.data = false;
      altura_objeto_pub.publish(bool_msg);

      // Estado de la pinza = "abierta"
      std_msgs::String pinza_msg;
      pinza_msg.data = "abierta";
      estado_pinza_pub.publish(pinza_msg);
    }

    // ---------------------------------------------------------------------------------
    // Lógica para imprimir colisión solo cuando hay cambio de estado o es el primer ciclo
    // ---------------------------------------------------------------------------------
    bool collision_state_changed = (current_collision_state != last_collision_state);
    bool print_now = (collision_state_changed || (first_loop && !current_collision_state));

    MyContactMap deepest_contacts;  // Almacena el contacto más profundo por par
    double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
    int count_fingers = 0;         // Cuenta de contactos con los dedos

    // Si hay colisión, hallar el contacto más profundo de cada par
    if (current_collision_state)
    {
      for (const auto &contact_pair : res.contacts)
      {
        double max_depth = -1e9;
        collision_detection::Contact best_contact;
        for (const auto &c : contact_pair.second)
        {
          if (c.depth > max_depth)
          {
            max_depth = c.depth;
            best_contact = c;
          }
        }
        deepest_contacts.insert({ contact_pair.first, { best_contact } });
      }

      // Solo calculamos la media si attach_state == false
      if (!attach_state)
      {
        for (const auto &dc_pair : deepest_contacts)
        {
          const std::string &obj1 = dc_pair.first.first;
          const std::string &obj2 = dc_pair.first.second;
          const collision_detection::Contact &bc = dc_pair.second.front();

          bool obj1IsFinger = (std::find(finger_objs.begin(), finger_objs.end(), obj1) != finger_objs.end());
          bool obj2IsFinger = (std::find(finger_objs.begin(), finger_objs.end(), obj2) != finger_objs.end());

          // Si al menos uno es un finger, sumamos sus coordenadas
          if (obj1IsFinger || obj2IsFinger)
          {
            sum_x += bc.pos.x();
            sum_y += bc.pos.y();
            sum_z += bc.pos.z();
            count_fingers++;
          }
        }
      }
    }

    // Decidir si imprimimos información de colisión (solo en cambios o primera vez sin colisión)
    if (print_now)
    {
      first_loop = false;

      if (current_collision_state)
      {
        // Transición a estado CON colisión
        std::cout << "\nCOLISION detectada." << std::endl;
        std::cout << "Número total de contactos: " << res.contact_count << "\n" << std::endl;

        // Imprimir detalles solo si no está attached
        if (!attach_state)
        {
          for (const auto &dc_pair : deepest_contacts)
          {
            const std::string &obj1 = dc_pair.first.first;
            const std::string &obj2 = dc_pair.first.second;
            const collision_detection::Contact &best_contact = dc_pair.second.front();

            bool obj1IsFinger = (std::find(finger_objs.begin(), finger_objs.end(), obj1) != finger_objs.end());
            bool obj2IsFinger = (std::find(finger_objs.begin(), finger_objs.end(), obj2) != finger_objs.end());

            // Solo interesa si alguno es un dedo
            if (!obj1IsFinger && !obj2IsFinger)
              continue;

            double depth_mm = best_contact.depth * 1000.0;
            std::cout << "  * Colisionan: " << obj1 << " <--> " << obj2
                      << " | Penetración MAX: "
                      << std::fixed << std::setprecision(3)
                      << depth_mm << " mm" << std::endl;

            std::cout << "     Punto más profundo (x,y,z): ["
                      << std::fixed << std::setprecision(3)
                      << best_contact.pos.x() << ", "
                      << best_contact.pos.y() << ", "
                      << best_contact.pos.z() << "]"
                      << std::endl;
          }

          // Calcular y mostrar la media de las coordenadas si hubo contactos con dedos
          if (count_fingers > 0)
          {
            double avg_x = sum_x / count_fingers;
            double avg_y = sum_y / count_fingers;
            double avg_z = sum_z / count_fingers;

            std::cout << "\n     >>> Media de las coordenadas (fingers): ["
                      << std::fixed << std::setprecision(3)
                      << avg_x << ", " << avg_y << ", " << avg_z << "]" << std::endl;

            last_media.x = round3(avg_x);
            last_media.y = round3(avg_y);
            last_media.z = round3(avg_z);
          }
          else
          {
            last_media.x = 0.0;
            last_media.y = 0.0;
            last_media.z = 0.0;
          }
        }
        else
        {
          // Si está attached => no mostramos contactos
          last_media.x = 0.0;
          last_media.y = 0.0;
          last_media.z = 0.0;
        }

        // Publicar marcadores de colisión
        {
          visualization_msgs::MarkerArray markers;
          std_msgs::ColorRGBA color;
          color.r = 1.0f;
          color.g = 0.0f;
          color.b = 1.0f;
          color.a = 0.5f;

          collision_detection::getCollisionMarkersFromContacts(
              markers,
              REFERENCE_FRAME,
              deepest_contacts,
              color,
              ros::Duration(),
              0.01
          );
          publishMarkers(markers);
        }
      }
      else
      {
        // Transición a estado SIN colisión
        std::cout << "\nSIN COLISION" << std::endl;
        // Borrar marcadores
        visualization_msgs::MarkerArray empty_markers;
        publishMarkers(empty_markers);

        // Media (0,0,0)
        last_media.x = 0.0;
        last_media.y = 0.0;
        last_media.z = 0.0;
      }
    }
    else
    {
      // No hay cambio de estado => actualizamos marcadores si sigue habiendo colisión
      if (current_collision_state)
      {
        // Crear marcadores de colisión
        visualization_msgs::MarkerArray markers;
        std_msgs::ColorRGBA color;
        color.r = 1.0f;
        color.g = 0.0f;
        color.b = 1.0f;
        color.a = 0.5f;

        collision_detection::getCollisionMarkersFromContacts(
            markers,
            REFERENCE_FRAME,
            deepest_contacts,
            color,
            ros::Duration(),
            0.01
        );
        publishMarkers(markers);

        // Recalcular la media si no está attached
        if (!attach_state && count_fingers > 0)
        {
          double avg_x = sum_x / count_fingers;
          double avg_y = sum_y / count_fingers;
          double avg_z = sum_z / count_fingers;

          last_media.x = round3(avg_x);
          last_media.y = round3(avg_y);
          last_media.z = round3(avg_z);
        }
        else
        {
          // attach_state == true => media (0,0,0)
          last_media.x = 0.0;
          last_media.y = 0.0;
          last_media.z = 0.0;
        }
      }
      else
      {
        // Sin colisión => borrar marcadores
        visualization_msgs::MarkerArray empty_markers;
        publishMarkers(empty_markers);

        last_media.x = 0.0;
        last_media.y = 0.0;
        last_media.z = 0.0;
      }
    }

    // ---------------------------------------------------------------------------------
    // Publicaciones continuas de datos al resto de nodos
    // ---------------------------------------------------------------------------------
    num_contactos_pub.publish(msg_num_contactos);
    media_contactos_pub.publish(last_media);

    // Actualizar la variable de estado de colisión y de attach
    last_collision_state = current_collision_state;
    attach_state_prev = attach_state;

    // ---------------------------------------------------------------------------------
    // Actualizar la posición de objetos manipulables en la escena en tiempo real
    // ---------------------------------------------------------------------------------
    for (const auto& object : objects_positions)
    {
        const std::string& name = object.first;
        const geometry_msgs::Pose& pose = object.second;

        moveit_msgs::CollisionObject moveObj;
        // Dependiendo del objeto, creamos la forma adecuada
        if (name == "unit_sphere")
        {
            moveObj = makeSphere(name, 0.035, pose.position.x, pose.position.y, pose.position.z);
        }
        else if (name == "unit_cylinder")
        {
            moveObj = makeCylinder(name, 0.1, 0.03, pose.position.x, pose.position.y, pose.position.z);
        }
        else
        {
            // Por defecto, creamos una caja (box)
            moveObj = makeBox(name, 0.05, 0.05, 0.05, pose.position.x, pose.position.y, pose.position.z);
        }

        // Publicar y actualizar en la escena de MoveIt
        collision_object_publisher.publish(moveObj);
        scene->processCollisionObjectMsg(moveObj);

        // Publicar la escena "diff"
        moveit_msgs::PlanningScene ps_msg;
        scene->getPlanningSceneMsg(ps_msg);
        ps_msg.is_diff = true;
        planning_scene_diff_pub.publish(ps_msg);
    }

    ros::spinOnce();
    rate.sleep();
  }

  ros::shutdown();
  return 0;
}
