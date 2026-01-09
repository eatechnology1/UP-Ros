<template>
  <div class="course-content">
    <!-- INTRODUCCIÓN -->
    <div class="section-group">
      <TextBlock>
        En Windows, cuando quieres un programa, buscas en Google, descargas un <code>.exe</code>, le
        das doble clic y rezas para no tener virus. <br /><br />
        En Linux, somos más civilizados. Usamos <strong>Gestores de Paquetes</strong>. Son como una
        "App Store" gigante, segura y gratuita que se controla desde la terminal.
      </TextBlock>
    </div>

    <!-- 1. SUDO: EL MODO DIOS -->
    <div class="section-group">
      <SectionTitle>1. "sudo": Permisos de Superusuario</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            Linux protege los archivos del sistema para que no los rompas por accidente. Si intentas
            instalar algo como usuario normal, te dirá: <em>"Permission denied"</em>. <br /><br />
            Para instalar, necesitamos poderes de administrador. Anteponemos la palabra
            <strong>sudo</strong>
            (SuperUser DO) al comando. Te pedirá tu contraseña (que no se ve mientras la escribes).
          </TextBlock>
        </template>
        <template #right>
          <AlertBlock type="danger" title="⚠️ Un gran poder...">
            Con <code>sudo</code> puedes borrar el sistema operativo entero. Úsalo solo cuando sepas
            qué estás haciendo.
            <br />
            Nunca uses <code>sudo</code> para comandos triviales como <code>ls</code> o
            <code>cd</code>.
          </AlertBlock>
        </template>
      </SplitBlock>
    </div>

    <!-- 2. APT: EL GESTOR DEL SISTEMA -->
    <div class="section-group">
      <SectionTitle>2. APT (Advanced Package Tool)</SectionTitle>
      <TextBlock>
        APT es el instalador principal de Ubuntu. Gestiona todo el software base, incluyendo ROS 2.
        El proceso tiene 3 pasos sagrados:
      </TextBlock>

      <div class="row q-col-gutter-md q-mt-sm">
        <div class="col-12">
          <StepsBlock
            :steps="[
              'Actualizar el catálogo (update): Le dice a tu PC qué versiones nuevas existen.',
              'Instalar (install): Descarga e instala el programa.',
              'Eliminar (remove): Desinstala el programa.',
            ]"
          />
        </div>
      </div>

      <div class="q-mt-md">
        <CodeBlock
          title="El ritual de instalación"
          lang="bash"
          content="# 1. Actualizar la lista de la tienda (SIEMPRE haz esto primero)
sudo apt update

# 2. Instalar un paquete (ej. el editor Nano)
sudo apt install nano

# 3. Eliminar un paquete
sudo apt remove nano"
          :copyable="true"
        />
      </div>

      <AlertBlock type="info" title="Instalando paquetes de ROS">
        Los paquetes de ROS siguen un patrón de nombre: <code>ros-[distro]-[paquete]</code>.
        <br />
        Ejemplo: <code>sudo apt install ros-jazzy-turtlesim</code>
      </AlertBlock>
    </div>

    <!-- 3. PIP: EL GESTOR DE PYTHON -->
    <div class="section-group">
      <SectionTitle>3. PIP (Python Installer Package)</SectionTitle>
      <TextBlock>
        APT instala programas del sistema. Pero en Robótica usamos muchas librerías específicas de
        Python (Cálculo matricial, Visión Artificial, IA) que a veces no están en APT o están
        desactualizadas.
        <br /><br />
        Para eso usamos <strong>pip</strong>. Es el instalador exclusivo para librerías de Python.
      </TextBlock>

      <SplitBlock>
        <template #left>
          <div class="text-h6 text-primary q-mb-sm">Comandos Clave</div>
          <CodeBlock
            lang="bash"
            content="# Instalar una librería
pip3 install numpy

# Instalar versión específica
pip3 install pandas==1.3.5

# Listar qué tienes instalado
pip3 list"
          />
        </template>
        <template #right>
          <div class="bg-dark q-pa-md rounded-borders full-height">
            <div class="text-subtitle2 text-white q-mb-sm">¿APT o PIP?</div>
            <p class="text-grey-4 text-body2">
              <strong>Regla de oro:</strong><br />
              1. Intenta instalar primero con <code>sudo apt install python3-nombre</code> (es más
              estable).<br />
              2. Si no existe, usa <code>pip3 install nombre</code>.<br /><br />
              En ROS 2, intentamos usar APT siempre que sea posible para evitar conflictos.
            </p>
          </div>
        </template>
      </SplitBlock>
    </div>

    <!-- 4. SOLUCIÓN DE PROBLEMAS -->
    <div class="section-group">
      <SectionTitle>4. Error Clásico: "Could not get lock"</SectionTitle>
      <TextBlock>
        A veces verás este error horrible:
        <br />
        <code>E: Could not get lock /var/lib/dpkg/lock-frontend</code>
      </TextBlock>

      <AlertBlock type="warning" title="¿Qué significa?">
        Significa que <strong>otra instalación está corriendo en segundo plano</strong> (quizás una
        actualización automática de Ubuntu). <br /><br />
        <strong>Solución:</strong> NO entres en pánico. Espera 5 minutos y vuelve a intentar. Si
        persiste, reinicia el PC.
      </AlertBlock>
    </div>

    <!-- RETO -->
    <div class="section-group">
      <SectionTitle>🏆 Reto Hacker</SectionTitle>
      <TextBlock>
        Vamos a instalar una aplicación inútil pero visualmente genial para que te sientas como en
        la película Matrix.
      </TextBlock>

      <CodeBlock
        title="Pasos del Reto"
        lang="bash"
        content="# 1. Actualiza tu catálogo
sudo apt update

# 2. Instala 'cmatrix'
sudo apt install cmatrix

# 3. Ejecútalo
cmatrix

# (Presiona 'q' o 'Ctrl+C' para salir de la Matrix)"
        :copyable="true"
      />

      <div class="q-mt-md text-center">
        <p class="text-grey-4">
          ¿Viste la lluvia de código verde? ¡Felicidades, ya sabes instalar software!
        </p>
      </div>
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
</style>
