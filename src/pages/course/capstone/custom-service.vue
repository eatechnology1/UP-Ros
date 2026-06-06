<template>
  <div class="course-content">
    <!-- INTRODUCCIÓN -->
    <div class="section-group">
      <div class="row items-center q-gutter-md">
        <div class="col-grow">
          <TextBlock>
            Los servicios estándar (`AddTwoInts`, `SetBool`) son útiles, pero limitados. En la vida
            real, tu robot necesita datos específicos.
            <br /><br />
            Hoy crearemos el servicio <strong>Descontaminar.srv</strong>.
            <br />
            - <strong>Petición:</strong> ¿Qué sector limpiar? (String) -
            <strong>Respuesta:</strong> ¿Fue exitoso? (Bool) y ¿Nivel de pureza logrado? (Int).
          </TextBlock>
        </div>
        <div class="col-auto">
          <div class="text-h1">☣️</div>
        </div>
      </div>
    </div>

    <!-- 1. CREANDO EL PAQUETE DE INTERFACES -->
    <div class="section-group">
      <SectionTitle>1. La Regla de la Separación</SectionTitle>

      <AlertBlock type="warning" title="¡Alto ahí!">
        Nunca mezcles definiciones de mensajes (`.msg`, `.srv`) en el mismo paquete que tu código
        Python.
        <br />
        Las interfaces necesitan compilarse con <strong>C++ (CMake)</strong> para ser universales.
        Si las metes en un paquete Python, tendrás pesadillas de dependencias circulares.
      </AlertBlock>

      <TextBlock>
        Vamos a crear un paquete dedicado exclusivamente a definir el "idioma" de nuestro robot.
      </TextBlock>

      <CodeBlock
        title="Creando my_robot_interfaces"
        lang="bash"
        content="cd ~/ros2_ws/src

# OJO: Usamos --build-type ament_cmake (NO python)
ros2 pkg create --build-type ament_cmake my_robot_interfaces

# Crear la carpeta para servicios
cd my_robot_interfaces
mkdir srv"
      />
    </div>

    <!-- 2. DEFINIENDO EL ARCHIVO .SRV -->
    <div class="section-group">
      <SectionTitle>2. Diseñando el Contrato (.srv)</SectionTitle>
      <TextBlock>
        Dentro de la carpeta `srv`, creamos el archivo <code>Descontaminar.srv</code>. Recuerda los
        tres guiones `---` que separan la pregunta de la respuesta.
      </TextBlock>

      <div class="srv-card q-my-md">
        <div class="srv-header">Descontaminar.srv</div>
        <div class="srv-content">
          <div class="comment"># REQUEST (Lo que pide el humano)</div>
          <div class="line">string sector_zona</div>
          <div class="line">int32 nivel_potencia</div>

          <div class="separator">---</div>

          <div class="comment"># RESPONSE (Lo que responde el robot)</div>
          <div class="line">bool exito</div>
          <div class="line">string mensaje_estado</div>
        </div>
      </div>
    </div>

    <!-- 3. CONFIGURACIÓN CMAKE (LA PARTE DIFICIL) -->
    <div class="section-group">
      <SectionTitle>3. La Burocracia del CMake</SectionTitle>
      <TextBlock>
        Para que ROS "cocine" este archivo y genere código Python/C++ automáticamente, debemos
        editar dos archivos sagrados. Respira hondo, esto es lo más técnico de ROS 2.
      </TextBlock>

      <SplitBlock>
        <template #left>
          <div class="file-edit">
            <div class="filename">package.xml</div>
            <p>Añade estas dependencias para que el generador funcione:</p>
            <CodeBlock
              lang="xml"
              content="<buildtool_depend>rosidl_default_generators</buildtool_depend>

<exec_depend>rosidl_default_runtime</exec_depend>

<member_of_group>rosidl_interface_packages</member_of_group>"
            />
          </div>
        </template>
        <template #right>
          <div class="file-edit">
            <div class="filename">CMakeLists.txt</div>
            <p>Busca la sección `find_package` y agrega esto ANTES de `ament_package()`:</p>
            <CodeBlock
              lang="cmake"
              content="find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  srv/Descontaminar.srv
)"
            />
          </div>
        </template>
      </SplitBlock>
    </div>

    <!-- 4. COMPILACIÓN -->
    <div class="section-group">
      <SectionTitle>4. Cocinando la Interface</SectionTitle>
      <TextBlock>
        Ahora debemos compilar este paquete para que ROS genere los módulos de Python invisibles.
      </TextBlock>

      <StepsBlock
        :steps="[
          'Ve a la raíz: cd ~/ros2_ws',
          'Compila solo este paquete: colcon build --packages-select my_robot_interfaces',
          'CRÍTICO: Haz source: source install/setup.bash',
          'Verifica que existe: ros2 interface show my_robot_interfaces/srv/Descontaminar',
        ]"
      />

      <div class="q-mt-md">
        <AlertBlock type="success" title="Si ves la estructura...">
          ¡Felicidades! Has extendido el lenguaje de ROS 2. Ahora cualquier nodo (Python, C++, Java)
          puede usar tu servicio.
        </AlertBlock>
      </div>
    </div>

    <!-- 5. IMPLEMENTACIÓN (SERVIDOR) -->
    <div class="section-group">
      <SectionTitle>5. Usando tu Creación</SectionTitle>
      <TextBlock>
        Ahora volvamos a nuestro paquete de Python (`my_bot_pkg`) y usemos la nueva importación.
        <br />
        <em
          >Nota: Debes añadir `<depend>my_robot_interfaces</depend>` en el package.xml de tu paquete
          Python.</em
        >
      </TextBlock>

      <CodeBlock
        title="nodo_limpieza.py"
        lang="python"
        content="import rclpy
from rclpy.node import Node
# ¡Aquí está tu creación!
from my_robot_interfaces.srv import Descontaminar

class LimpiezaService(Node):
    def __init__(self):
        super().__init__('limpieza_service')
        self.srv = self.create_service(
            Descontaminar,
            'descontaminar_sector',
            self.limpiar_callback
        )

    def limpiar_callback(self, request, response):
        zona = request.sector_zona

        self.get_logger().info(f'☢️ Iniciando purga en {zona} al {request.nivel_potencia}%...')

        if request.nivel_potencia > 80:
            response.exito = True
            response.mensaje_estado = 'Sector PURIFICADO. Bacterias: 0%'
        else:
            response.exito = False
            response.mensaje_estado = 'Fallo: Potencia insuficiente para matar bacterias.'

        return response"
        :copyable="true"
      />
    </div>

    <!-- RETO -->
    <div class="section-group">
      <SectionTitle>🏆 Reto de Integración</SectionTitle>
      <TextBlock>
        Este es un reto de "Full Stack ROS":
        <br /><br />
        1. Crea el paquete de interfaces y el servicio `.srv`. 2. Compílalo y verifica con `ros2
        interface show`. 3. Crea el nodo servidor en Python. 4. Llama al servicio desde la terminal:
        <br />
        <code>ros2 service call /descontaminar_sector ...</code>
        <br /><br />
        Si logras que el robot te responda "Sector PURIFICADO", habrás dominado la comunicación
        customizada.
      </TextBlock>
    </div>
  </div>
</template>

<script setup lang="ts">
import SectionTitle from 'components/content/SectionTitle.vue';
import TextBlock from 'components/content/TextBlock.vue';
import AlertBlock from 'components/content/AlertBlock.vue';
import CodeBlock from 'components/content/CodeBlock.vue';
import SplitBlock from 'components/content/SplitBlock.vue';
import StepsBlock from 'components/content/StepsBlock.vue';
</script>

<style scoped>
.section-group {
  margin-bottom: 3rem;
}

/* Visualización .SRV Card */
.srv-card {
  background: var(--bg-surface-hover);
  border-left: 5px solid #a855f7; /* Purple */
  border-radius: 8px;
  overflow: hidden;
  box-shadow: 0 4px 6px rgba(0, 0, 0, 0.3);
  max-width: 500px;
  margin: 20px auto;
}
.srv-header {
  background: var(--bg-surface-solid);
  color: var(--text-secondary);
  padding: 8px 16px;
  font-weight: bold;
  font-family: 'Fira Code', monospace;
  font-size: 0.9rem;
}
.srv-content {
  padding: 16px;
  font-family: 'Fira Code', monospace;
  font-size: 0.95rem;
  color: #a5f3fc;
}
.srv-content .line {
  margin-bottom: 4px;
}
.srv-content .comment {
  color: #64748b;
  margin-top: 8px;
  margin-bottom: 4px;
  font-style: italic;
}
.srv-content .separator {
  color: #f472b6;
  font-weight: bold;
  margin: 10px 0;
  letter-spacing: 3px;
}

/* File Edit Visuals */
.file-edit {
  background: var(--bg-surface-hover);
  border: 1px solid #475569;
  border-radius: 8px;
  padding: 15px;
  height: 100%;
}
.file-edit .filename {
  color: var(--text-warning, #d97706);
  font-weight: bold;
  border-bottom: 1px solid #475569;
  margin-bottom: 10px;
  padding-bottom: 5px;
}
.file-edit p {
  font-size: 0.85rem;
  color: var(--text-secondary);
}
</style>
