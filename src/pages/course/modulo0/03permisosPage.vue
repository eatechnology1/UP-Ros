<template>
  <q-page class="q-pa-lg column items-center">
    <!-- 1. HERO SECTION -->
    <section class="intro-hero self-stretch">
      <div class="hero-content">
        <div class="text-overline text-accent text-weight-bold q-mb-sm">
          MÓDULO 0: FUNDAMENTOS DEL SISTEMA
        </div>

        <h1 class="hero-title">Jerarquías y <span class="text-primary">Control de Acceso</span></h1>

        <TextBlock>
          Linux no es una democracia; es una meritocracia militarizada. Aprende quién tiene la llave
          nuclear (Root) y cómo autorizar a tu robot para acceder al hardware sin comprometer la
          seguridad del sistema.
        </TextBlock>
      </div>
    </section>

    <!-- 2. LA METÁFORA -->
    <div class="section-group self-stretch">
      <SectionTitle>1. El Sistema Inmune Digital</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            Imagina tu sistema como un laboratorio de bioseguridad. No dejarías que cualquiera entre
            a la sala de virus.
            <br /><br />
            Cada archivo y dispositivo (como un puerto USB) tiene un guardia que revisa tu
            credencial (Usuario/Grupo) para decidir si puedes:
          </TextBlock>
          <ul class="tool-list q-pl-md">
            <li>👀 <strong>Leer (r):</strong> Mirar sin tocar.</li>
            <li>✍️ <strong>Escribir (w):</strong> Modificar el experimento.</li>
            <li>🚀 <strong>Ejecutar (x):</strong> Iniciar la maquinaria.</li>
          </ul>
        </template>

        <template #right>
          <!-- Tarjeta de Jerarquía Personalizada -->
          <div class="tool-card hierarchy">
            <div class="tool-header q-mb-md">
              <q-icon name="security" size="sm" color="white" />
              <h4 class="text-h6 text-white q-my-none">Niveles de Acceso</h4>
            </div>

            <div class="q-gutter-y-sm">
              <div class="bg-red-9 text-white q-pa-sm rounded-borders row items-center">
                <q-icon name="warning" class="q-mr-sm" />
                <div>
                  <strong>ROOT (Sudo)</strong>
                  <div class="text-caption">Dios del sistema. Crea y destruye todo.</div>
                </div>
              </div>

              <div class="bg-blue-9 text-white q-pa-sm rounded-borders row items-center q-ml-md">
                <q-icon name="engineering" class="q-mr-sm" />
                <div>
                  <strong>USUARIO (Tú)</strong>
                  <div class="text-caption">Dueño de su /home y archivos propios.</div>
                </div>
              </div>

              <div class="bg-grey-8 text-grey-4 q-pa-sm rounded-borders row items-center q-ml-xl">
                <q-icon name="group" class="q-mr-sm" />
                <div>
                  <strong>OTROS (Others)</strong>
                  <div class="text-caption">Visitantes. Acceso restringido.</div>
                </div>
              </div>
            </div>
          </div>
        </template>
      </SplitBlock>
    </div>

    <!-- 3. DECODIFICANDO LS -L -->
    <div class="section-group self-stretch">
      <SectionTitle>2. La Matriz de Permisos</SectionTitle>
      <TextBlock>
        Al ejecutar <code>ls -l</code>, Linux muestra un código críptico (<code>-rwxr-xr--</code>).
        Diseccionémoslo.
      </TextBlock>

      <!-- Decoder Visual -->
      <div class="permission-decoder q-my-lg tool-card no-border-top">
        <div class="text-h3 font-mono text-center text-white q-mb-md">
          <span class="text-grey-6">-</span><span class="text-green-4">rwx</span
          ><span class="text-blue-4">r-x</span><span class="text-red-4">r--</span>
        </div>

        <div class="row text-center text-caption font-mono q-col-gutter-sm">
          <div class="col-1 text-grey-5 flex flex-center">Tipo</div>
          <div class="col-3 text-green-4 border-r">
            <strong>PROPIETARIO</strong><br />(Lectura+Escritura+Ejecución)
          </div>
          <div class="col-4 text-blue-4 border-r">
            <strong>GRUPO</strong><br />(Solo Lectura+Ejecución)
          </div>
          <div class="col-4 text-red-4"><strong>OTROS</strong><br />(Solo Lectura)</div>
        </div>
      </div>
    </div>

    <!-- 4. CHMOD (Matemática Octal) -->
    <div class="section-group self-stretch">
      <SectionTitle>3. chmod: Matemática Octal</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock> Los "sysadmins" usamos números. La suma lo es todo: </TextBlock>
          <div class="row q-gutter-md q-my-sm font-mono text-subtitle1">
            <div class="text-green-4">4 = Leer (r)</div>
            <div class="text-orange-4">2 = Escribir (w)</div>
            <div class="text-red-4">1 = Ejecutar (x)</div>
          </div>
          <TextBlock>
            • <strong>7</strong> (4+2+1): Acceso Total.<br />
            • <strong>5</strong> (4+0+1): Leer y Ejecutar.<br />
            • <strong>6</strong> (4+2+0): Leer y Escribir.
          </TextBlock>
        </template>

        <template #right>
          <AlertBlock type="warning" title="☢️ El peligroso 777">
            Jamás uses <code>chmod 777</code>. Darle permiso de escritura a "Todo el mundo" es dejar
            la puerta abierta a ataques.
          </AlertBlock>
          <CodeBlock title="Ejemplo Seguro" lang="bash" content="chmod 755 mi_script_robot.py" />
          <div class="text-caption text-grey-5 q-mt-xs font-mono text-center">
            7(Yo) | 5(Grupo) | 5(Mundo)
          </div>
        </template>
      </SplitBlock>
    </div>

    <!-- 5. SUDO -->
    <div class="section-group self-stretch">
      <SectionTitle>4. SUDO (SuperUser DO)</SectionTitle>
      <TextBlock>
        Es invocar poderes de administrador temporalmente. Úsalo con sabiduría.
      </TextBlock>

      <div class="row q-gutter-md q-my-md justify-center">
        <q-chip color="dark" text-color="yellow-8" icon="vpn_key">sudo apt update</q-chip>
        <q-chip color="dark" text-color="yellow-8" icon="power_settings_new">sudo reboot</q-chip>
      </div>

      <AlertBlock type="danger" title="💀 Pecado Capital en ROS">
        <strong>NUNCA compiles con sudo (colcon build).</strong>
        <br />
        Si lo haces, los archivos generados pertenecerán a Root y tu usuario normal no podrá
        sobreescribirlos luego, rompiendo tu workspace.
      </AlertBlock>
    </div>

    <!-- 6. EL PROBLEMA DEL HARDWARE -->
    <div class="section-group self-stretch q-mb-xl">
      <SectionTitle>5. Caso Real: Acceso a Hardware</SectionTitle>

      <div class="tool-card dialout">
        <div class="row items-center q-mb-md">
          <q-icon name="usb" color="accent" size="md" class="q-mr-md" />
          <h3 class="text-h6 text-white q-my-none">Error: "Permission Denied" en /dev/ttyUSB0</h3>
        </div>

        <SplitBlock>
          <template #left>
            <TextBlock>
              Rito de iniciación: Conectas tu Arduino/LIDAR y falla.
              <br /><br />
              En Linux, el hardware pertenece al grupo <strong>dialout</strong>. Tu usuario no está
              en la lista VIP por defecto.
            </TextBlock>
          </template>
          <template #right>
            <CodeBlock
              title="Solución Definitiva"
              lang="bash"
              content="# 1. Agrégate al grupo de hardware
sudo usermod -aG dialout $USER

# 2. IMPORTANTE: Reinicia el PC para aplicar"
            />
          </template>
        </SplitBlock>
      </div>
    </div>
  </q-page>
</template>

<script setup lang="ts">
import TextBlock from 'components/content/TextBlock.vue';
import AlertBlock from 'components/content/AlertBlock.vue';
import CodeBlock from 'components/content/CodeBlock.vue';
import SectionTitle from 'components/content/SectionTitle.vue';
import SplitBlock from 'components/content/SplitBlock.vue';
</script>

<style scoped>
/* --- ESTILOS MAESTROS --- */
.intro-hero,
.section-group {
  width: 100%;
  max-width: 1100px;
  margin: 0 auto 3.5rem auto;
}

.intro-hero {
  padding: 3rem 2rem;
  background:
    radial-gradient(circle at center, rgba(236, 72, 153, 0.15), transparent 60%),
    rgba(15, 23, 42, 0.8);
  backdrop-filter: blur(20px);
  border-radius: 24px;
  border: 1px solid rgba(148, 163, 184, 0.2);
  text-align: center;
}

.hero-title {
  font-size: 3rem;
  font-weight: 800;
  margin: 0 0 1.5rem 0;
  line-height: 1.1;
  color: #f8fafc;
}

/* TOOL CARDS GENÉRICAS */
.tool-card {
  padding: 24px;
  border-radius: 16px;
  background: rgba(30, 41, 59, 0.4);
  border: 1px solid rgba(255, 255, 255, 0.05);
}

.tool-card.hierarchy {
  border-top: 4px solid #ef4444;
} /* Red for Root/Security */
.tool-card.dialout {
  border-top: 4px solid #f472b6;
} /* Accent Pink */
.tool-card.no-border-top {
  border-top: none;
  border: 1px solid rgba(148, 163, 184, 0.2);
}

.tool-list {
  list-style: none;
  padding: 0;
  margin-top: 16px;
  color: #cbd5e1;
}
.tool-list li {
  margin-bottom: 8px;
}

/* UTILS */
.font-mono {
  font-family: 'Fira Code', monospace;
}

.border-r {
  border-right: 1px dashed rgba(255, 255, 255, 0.15);
}

@media (max-width: 768px) {
  .hero-title {
    font-size: 2.2rem;
  }
}
</style>
