<template>
  <q-page class="installation-page">

    <!-- ╔══════════════════════════════════════════════╗ -->
    <!-- ║              HERO ÉPICO                      ║ -->
    <!-- ╚══════════════════════════════════════════════╝ -->
    <section class="install-hero">
      <div class="hero-particles">
        <div v-for="i in 12" :key="i" class="particle" :style="particleStyle(i)"></div>
      </div>

      <div class="hero-inner">
        <div class="hero-badge animate-fade-in">
          <q-icon name="terminal" size="xs" />
          Módulo 0 · Configuración del Entorno
        </div>

        <h1 class="hero-h1 animate-slide-up">
          Tu Primer Paso al Mundo<br />
          <span class="text-gradient-hero">de la Robótica Profesional</span>
        </h1>

        <p class="hero-sub animate-slide-up-delay">
          Vamos a instalar <strong>ROS 2 Jazzy</strong>, el sistema nervioso de los robots modernos.
          No necesitas experiencia previa — te guiamos desde el principio.
        </p>

        <div class="hero-stats animate-fade-in-delay">
          <div class="stat-chip">
            <q-icon name="schedule" size="xs" />
            45–60 min
          </div>
          <div class="stat-chip">
            <q-icon name="inventory_2" size="xs" />
            ~500 paquetes
          </div>
          <div class="stat-chip">
            <q-icon name="verified" size="xs" />
            LTS hasta 2029
          </div>
          <div class="stat-chip">
            <q-icon name="school" size="xs" />
            Sin requisitos previos
          </div>
        </div>
      </div>
    </section>

    <div class="page-body">

      <!-- ╔══════════════════════════════════════════════╗ -->
      <!-- ║    ¿QUÉ VAMOS A INSTALAR?                    ║ -->
      <!-- ╚══════════════════════════════════════════════╝ -->
      <section class="page-section">
        <SectionTitle>¿Qué vamos a instalar exactamente?</SectionTitle>
        <TextBlock>
          Antes de correr comandos, entendamos <strong>qué es cada herramienta</strong>.
          No te preocupes por memorizar esto, solo necesitas tener el contexto.
        </TextBlock>

        <div class="explain-grid q-mt-lg">
          <div class="explain-card" v-for="item in whatWeInstall" :key="item.title">
            <div class="explain-icon" :style="{ background: item.color + '22', borderColor: item.color + '44' }">
              <q-icon :name="item.icon" size="lg" :style="{ color: item.color }" />
            </div>
            <div class="explain-body">
              <div class="explain-title">{{ item.title }}</div>
              <div class="explain-version">{{ item.version }}</div>
              <div class="explain-desc">{{ item.desc }}</div>
              <div class="explain-analogy">
                <q-icon name="lightbulb" size="xs" color="amber" />
                {{ item.analogy }}
              </div>
            </div>
          </div>
        </div>
      </section>

      <!-- ╔══════════════════════════════════════════════╗ -->
      <!-- ║    ¿EN QUÉ SISTEMA ESTÁS?                    ║ -->
      <!-- ╚══════════════════════════════════════════════╝ -->
      <section class="page-section">
        <SectionTitle>¿En qué sistema estás trabajando?</SectionTitle>
        <TextBlock>
          ROS 2 funciona de forma nativa en <strong>Linux (Ubuntu)</strong>. Si tienes Windows o macOS,
          también tienes opciones — elige tu situación:
        </TextBlock>

        <div class="platform-grid q-mt-lg">
          <div
            v-for="p in platforms"
            :key="p.id"
            class="platform-card"
            :class="{ 'platform-selected': selectedPlatform === p.id }"
            @click="selectedPlatform = p.id"
          >
            <div class="platform-icon">
              <q-icon :name="p.icon" size="xl" :color="p.color" />
            </div>
            <div class="platform-name">{{ p.name }}</div>
            <div class="platform-sub">{{ p.subtitle }}</div>
            <q-badge v-if="p.recommended" class="platform-badge" color="positive">
              Recomendado
            </q-badge>
            <q-badge v-if="selectedPlatform === p.id" class="platform-selected-badge" color="primary">
              Seleccionado
            </q-badge>
          </div>
        </div>

        <!-- Instrucciones por plataforma -->
        <transition name="fade" mode="out-in">
          <AlertBlock v-if="selectedPlatform === 'native'" type="success" title="Ubuntu 24.04 Nativo — Perfecto" class="q-mt-md">
            Tienes la configuración ideal. Todos los pasos de esta guía aplican directamente.
            Solo asegúrate de tener Ubuntu <strong>24.04</strong> (no 22.04 ni 20.04).
          </AlertBlock>

          <div v-else-if="selectedPlatform === 'vm'" class="vm-guide q-mt-md" key="vm">
            <AlertBlock type="info" title="Máquina Virtual (VirtualBox / VMware)">
              Puedes instalar Ubuntu 24.04 como máquina virtual. Es la opción más común si tienes
              Windows o macOS. <strong>Requiere habilitar virtualización en la BIOS</strong> y al menos
              <strong>8GB de RAM y 40GB de disco</strong> libres para la VM.
            </AlertBlock>
            <div class="vm-steps q-mt-md">
              <div class="vm-step" v-for="(s, i) in vmSteps" :key="i">
                <div class="vm-step-num">{{ i + 1 }}</div>
                <div class="vm-step-text" v-html="s"></div>
              </div>
            </div>
          </div>

          <div v-else-if="selectedPlatform === 'wsl2'" class="q-mt-md" key="wsl2">
            <AlertBlock type="warning" title="WSL2 (Windows Subsystem for Linux)">
              WSL2 funciona para tareas básicas de ROS 2, pero tiene limitaciones con
              herramientas gráficas como <strong>RViz2 y Gazebo</strong>. Para este curso,
              <strong>recomendamos una VM o instalación nativa</strong>.
            </AlertBlock>
          </div>
        </transition>
      </section>

      <!-- ╔══════════════════════════════════════════════╗ -->
      <!-- ║    ¿CÓMO ABRO UNA TERMINAL?                  ║ -->
      <!-- ╚══════════════════════════════════════════════╝ -->
      <section class="page-section">
        <SectionTitle>¿Cómo abro una Terminal?</SectionTitle>
        <TextBlock>
          La <strong>terminal</strong> es tu herramienta principal. Piénsala como el "modo texto" de tu
          computador: en lugar de hacer clic en íconos, escribes instrucciones directas. Es mucho más
          poderosa de lo que parece.
        </TextBlock>

        <div class="terminal-guide q-mt-lg">
          <div class="tg-method" v-for="m in terminalMethods" :key="m.key">
            <div class="tg-icon">
              <q-icon :name="m.icon" size="md" color="primary" />
            </div>
            <div class="tg-content">
              <div class="tg-title">{{ m.title }}</div>
              <div class="tg-desc">{{ m.desc }}</div>
              <div class="tg-shortcut" v-if="m.shortcut">
                <q-icon name="keyboard" size="xs" />
                {{ m.shortcut }}
              </div>
            </div>
          </div>
        </div>

        <!-- Mock de terminal -->
        <div class="terminal-mock q-mt-lg">
          <div class="terminal-mock-bar">
            <span class="dot red"></span><span class="dot yellow"></span><span class="dot green"></span>
            <span class="terminal-mock-title">Terminal — usuario@ubuntu:~</span>
          </div>
          <div class="terminal-mock-body">
            <div class="t-line"><span class="t-prompt">usuario@ubuntu:~$</span> <span class="t-cmd">lsb_release -a</span></div>
            <div class="t-line t-output">No LSB modules are available.</div>
            <div class="t-line t-output">Distributor ID: Ubuntu</div>
            <div class="t-line t-output t-highlight">Description:    Ubuntu 24.04.1 LTS</div>
            <div class="t-line t-output">Release:        24.04</div>
            <div class="t-line t-output t-highlight">Codename:       noble</div>
            <div class="t-line"><span class="t-prompt">usuario@ubuntu:~$</span> <span class="t-cursor">█</span></div>
          </div>
        </div>
        <div class="mock-caption">Así se ve la terminal con el resultado esperado del primer comando</div>
      </section>

      <!-- ╔══════════════════════════════════════════════╗ -->
      <!-- ║    PRE-FLIGHT CHECKLIST                      ║ -->
      <!-- ╚══════════════════════════════════════════════╝ -->
      <section class="page-section">
        <SectionTitle>Pre-Flight Checklist ✈️</SectionTitle>
        <TextBlock>
          Antes de instalar, verifica que tienes todo listo. Haz clic en cada ítem al confirmarlo:
        </TextBlock>

        <div class="preflight-wrapper q-mt-lg">
          <div class="preflight-items">
            <div
              v-for="(item, idx) in preflightItems"
              :key="idx"
              class="preflight-item"
              :class="{ 'item-checked': item.checked }"
              @click="item.checked = !item.checked"
            >
              <div class="pf-check">
                <q-icon :name="item.checked ? 'check_circle' : 'radio_button_unchecked'"
                  :color="item.checked ? 'positive' : 'grey-5'" size="md" />
              </div>
              <div class="pf-body">
                <div class="pf-label" :class="{ 'pf-done': item.checked }">{{ item.label }}</div>
                <div class="pf-detail">
                  <q-icon name="terminal" size="xs" class="q-mr-xs" />
                  <code>{{ item.verify }}</code>
                </div>
                <div class="pf-hint">{{ item.hint }}</div>
              </div>
              <div class="pf-status" v-if="item.checked">
                <q-badge color="positive" outline>OK</q-badge>
              </div>
            </div>
          </div>

          <div class="preflight-footer">
            <div class="pf-progress-label">
              <span>Progreso: <strong>{{ checkedCount }}/{{ preflightItems.length }}</strong></span>
              <span class="pf-pct">{{ preflightProgress }}%</span>
            </div>
            <q-linear-progress
              :value="preflightProgress / 100"
              :color="preflightProgress === 100 ? 'positive' : 'primary'"
              size="14px"
              rounded
              class="q-mt-sm"
            />
            <transition name="fade">
              <div v-if="preflightProgress === 100" class="pf-ready">
                <q-icon name="rocket_launch" color="positive" />
                ¡Todo listo! Puedes comenzar la instalación.
              </div>
            </transition>
          </div>
        </div>
      </section>

      <!-- ╔══════════════════════════════════════════════╗ -->
      <!-- ║    ROADMAP VISUAL DE PASOS                   ║ -->
      <!-- ╚══════════════════════════════════════════════╝ -->
      <section class="page-section">
        <SectionTitle>Mapa de la Instalación</SectionTitle>
        <TextBlock>
          Son <strong>7 pasos</strong> en total. Cada uno construye sobre el anterior.
          Haz clic en cualquiera para ir directo a ese paso.
        </TextBlock>

        <div class="roadmap-steps q-mt-lg">
          <div
            v-for="(step, idx) in installSteps"
            :key="idx"
            class="rmap-step"
            :class="{ 'rmap-active': activeStep === idx }"
            @click="scrollToStep(idx)"
          >
            <div class="rmap-num" :style="{ background: step.color + '33', borderColor: step.color }">
              <q-icon :name="step.icon" size="sm" :style="{ color: step.color }" />
            </div>
            <div class="rmap-info">
              <div class="rmap-title">{{ step.title }}</div>
              <div class="rmap-time">{{ step.time }}</div>
            </div>
            <div v-if="idx < installSteps.length - 1" class="rmap-connector"></div>
          </div>
        </div>
      </section>

      <!-- ╔══════════════════════════════════════════════╗ -->
      <!-- ║    PASO 1: VERIFICAR OS                      ║ -->
      <!-- ╚══════════════════════════════════════════════╝ -->
      <section class="page-section step-section" :ref="el => setStepRef(el, 0)">
        <div class="step-header" :style="{ borderColor: stepColor(0) }">
          <div class="step-num-badge" :style="{ background: stepColor(0) }">1</div>
          <div>
            <div class="step-label">Paso 1</div>
            <div class="step-title">Verificar el Sistema Operativo</div>
          </div>
          <div class="step-time-badge">
            <q-icon name="schedule" size="xs" /> {{ stepTime(0) }}
          </div>
        </div>

        <AlertBlock type="warning" title="Requisito Crítico" class="q-mt-md q-mb-md">
          ROS 2 Jazzy <strong>SOLO</strong> es compatible con Ubuntu 24.04 (Noble Numbat).
          Cualquier otra versión provocará errores de dependencias imposibles de resolver sin reinstalar.
        </AlertBlock>

        <SplitBlock>
          <template #left>
            <TextBlock>
              Ejecuta este comando en tu terminal. Busca que la salida diga
              <strong>"24.04"</strong> y <strong>"noble"</strong> como en el ejemplo de abajo.
            </TextBlock>

            <div class="compat-table q-mt-md">
              <div class="ct-row ct-header">
                <span>Ubuntu</span><span>ROS 2</span><span>Estado</span>
              </div>
              <div class="ct-row ct-ok">
                <span>24.04 Noble</span><span>Jazzy</span><span class="ct-badge ok">✅ LTS 2029</span>
              </div>
              <div class="ct-row ct-warn">
                <span>22.04 Jammy</span><span>Humble</span><span class="ct-badge warn">⚠️ Anterior</span>
              </div>
              <div class="ct-row ct-bad">
                <span>20.04 Focal</span><span>Foxy</span><span class="ct-badge bad">❌ EOL</span>
              </div>
              <div class="ct-row ct-bad">
                <span>Windows / macOS</span><span>—</span><span class="ct-badge bad">❌ Directo</span>
              </div>
            </div>
          </template>

          <template #right>
            <CodeBlock
              title="Verificar versión de Ubuntu"
              lang="bash"
              :content="cmd.osCheck"
              :copyable="true"
            />
            <div class="expected-output q-mt-md">
              <div class="eo-label">
                <q-icon name="check_circle" color="positive" size="xs" />
                Resultado esperado:
              </div>
              <div class="eo-content">
                <span>Description: <strong class="eo-highlight">Ubuntu 24.04.x LTS</strong></span><br/>
                <span>Codename: <strong class="eo-highlight">noble</strong></span>
              </div>
            </div>
          </template>
        </SplitBlock>
      </section>

      <!-- ╔══════════════════════════════════════════════╗ -->
      <!-- ║    PASO 2: LOCALE UTF-8                      ║ -->
      <!-- ╚══════════════════════════════════════════════╝ -->
      <section class="page-section step-section" :ref="el => setStepRef(el, 1)">
        <div class="step-header" :style="{ borderColor: stepColor(1) }">
          <div class="step-num-badge" :style="{ background: stepColor(1) }">2</div>
          <div>
            <div class="step-label">Paso 2</div>
            <div class="step-title">Configurar Locale (UTF-8)</div>
          </div>
          <div class="step-time-badge">
            <q-icon name="schedule" size="xs" /> {{ stepTime(1) }}
          </div>
        </div>

        <q-expansion-item
          class="why-expansion q-mb-md"
          icon="help_outline"
          label="¿Por qué necesitamos este paso?"
          expand-separator
          header-class="why-header"
        >
          <div class="why-body">
            <p>
              <strong>UTF-8</strong> es el sistema de codificación de texto que permite a los programas
              leer caracteres especiales (tildes, chino, árabe, etc.). ROS 2 usa UTF-8 para intercambiar
              mensajes entre nodos. Sin esto, los caracteres en los mensajes pueden corromperse y causar
              errores difíciles de diagnosticar.
            </p>
            <p>
              Piénsalo como el <em>"idioma común"</em> entre todas las partes de ROS 2.
            </p>
          </div>
        </q-expansion-item>

        <CodeBlock
          title="Configurar UTF-8"
          lang="bash"
          :content="cmd.locale"
          :copyable="true"
        />

        <div class="expected-output q-mt-md">
          <div class="eo-label">
            <q-icon name="check_circle" color="positive" size="xs" />
            Resultado esperado al final:
          </div>
          <div class="eo-content">
            <span>LANG=<strong class="eo-highlight">en_US.UTF-8</strong></span><br/>
            <span>LC_ALL=<strong class="eo-highlight">en_US.UTF-8</strong></span>
          </div>
        </div>
      </section>

      <!-- ╔══════════════════════════════════════════════╗ -->
      <!-- ║    PASO 3: REPOSITORIOS                      ║ -->
      <!-- ╚══════════════════════════════════════════════╝ -->
      <section class="page-section step-section" :ref="el => setStepRef(el, 2)">
        <div class="step-header" :style="{ borderColor: stepColor(2) }">
          <div class="step-num-badge" :style="{ background: stepColor(2) }">3</div>
          <div>
            <div class="step-label">Paso 3</div>
            <div class="step-title">Agregar Repositorios Oficiales</div>
          </div>
          <div class="step-time-badge">
            <q-icon name="schedule" size="xs" /> {{ stepTime(2) }}
          </div>
        </div>

        <q-expansion-item
          class="why-expansion q-mb-md"
          icon="help_outline"
          label="¿Qué es un repositorio y por qué necesito uno?"
          expand-separator
          header-class="why-header"
        >
          <div class="why-body">
            <p>
              Un <strong>repositorio</strong> es como una tienda de aplicaciones, pero en modo texto.
              Ubuntu ya tiene su propia tienda (el repositorio oficial de Ubuntu), pero los paquetes
              de ROS 2 no están ahí — están en los servidores de <strong>packages.ros.org</strong>.
            </p>
            <p>
              Este paso le dice a Ubuntu: <em>"también mira en este otro lugar cuando busques paquetes"</em>.
              Sin esto, cuando intentes instalar ROS 2, Ubuntu responderá con
              <code>"Unable to locate package"</code>.
            </p>
          </div>
        </q-expansion-item>

        <div class="mini-steps q-mb-md">
          <div class="mini-step" v-for="(ms, i) in repoMiniSteps" :key="i">
            <div class="ms-num">{{ i + 1 }}</div>
            <div class="ms-text" v-html="ms"></div>
          </div>
        </div>

        <CodeBlock
          title="Configurar Repositorios de ROS 2"
          lang="bash"
          :content="cmd.repos"
          :copyable="true"
        />

        <AlertBlock type="info" title="Si el comando tarda mucho…" class="q-mt-md">
          La descarga del paquete de configuración depende de tu conexión a internet. Si tarda más de
          5 minutos, verifica tu conexión o intenta de nuevo.
        </AlertBlock>
      </section>

      <!-- ╔══════════════════════════════════════════════╗ -->
      <!-- ║    PASO 4: INSTALAR ROS 2                    ║ -->
      <!-- ╚══════════════════════════════════════════════╝ -->
      <section class="page-section step-section" :ref="el => setStepRef(el, 3)">
        <div class="step-header" :style="{ borderColor: stepColor(3) }">
          <div class="step-num-badge" :style="{ background: stepColor(3) }">4</div>
          <div>
            <div class="step-label">Paso 4</div>
            <div class="step-title">Instalar ROS 2 Jazzy</div>
          </div>
          <div class="step-time-badge">
            <q-icon name="schedule" size="xs" /> {{ stepTime(3) }}
          </div>
        </div>

        <TextBlock class="q-mb-lg">
          Elige la variante que mejor se ajuste a tus necesidades.
          Para este curso recomendamos <strong>Desktop Full</strong>.
        </TextBlock>

        <div class="variant-selector q-mb-lg">
          <div
            v-for="v in variants"
            :key="v.id"
            class="variant-card"
            :class="{ 'variant-selected': selectedVariant === v.id }"
            @click="selectedVariant = v.id"
          >
            <div class="vc-top">
              <q-icon :name="v.icon" size="lg" :color="v.color" />
              <div class="vc-sel" v-if="selectedVariant === v.id">
                <q-icon name="check_circle" color="positive" />
              </div>
            </div>
            <div class="vc-name">{{ v.name }}</div>
            <div class="vc-pkg"><code>{{ v.pkg }}</code></div>
            <div class="vc-size">
              <q-icon name="download" size="xs" /> {{ v.size }}
            </div>
            <ul class="vc-includes">
              <li v-for="inc in v.includes" :key="inc">
                <q-icon name="check" size="xs" color="positive" /> {{ inc }}
              </li>
            </ul>
            <q-badge v-if="v.recommended" color="positive" class="vc-rec">Recomendado</q-badge>
          </div>
        </div>

        <SplitBlock>
          <template #left>
            <div class="install-info-box">
              <div class="iib-item">
                <q-icon name="info" color="primary" size="sm" />
                <div>
                  <div class="iib-title">¿Cuánto tiempo tarda?</div>
                  <div class="iib-desc">Entre 10 y 25 minutos dependiendo de tu conexión</div>
                </div>
              </div>
              <div class="iib-item">
                <q-icon name="storage" color="primary" size="sm" />
                <div>
                  <div class="iib-title">Espacio en disco</div>
                  <div class="iib-desc">Desktop Full requiere ~3GB instalado</div>
                </div>
              </div>
              <div class="iib-item">
                <q-icon name="security" color="warning" size="sm" />
                <div>
                  <div class="iib-title">Contraseña sudo</div>
                  <div class="iib-desc">Se te pedirá la contraseña de administrador</div>
                </div>
              </div>
              <div class="iib-item">
                <q-icon name="do_not_disturb" color="positive" size="sm" />
                <div>
                  <div class="iib-title">¿Qué hago mientras espera?</div>
                  <div class="iib-desc">¡Nada! Solo deja la terminal abierta y espera</div>
                </div>
              </div>
            </div>
          </template>
          <template #right>
            <CodeBlock
              title="Instalar ROS 2"
              lang="bash"
              :content="installCommand"
              :copyable="true"
            />
          </template>
        </SplitBlock>
      </section>

      <!-- ╔══════════════════════════════════════════════╗ -->
      <!-- ║    PASO 5: HERRAMIENTAS DE DESARROLLO        ║ -->
      <!-- ╚══════════════════════════════════════════════╝ -->
      <section class="page-section step-section" :ref="el => setStepRef(el, 4)">
        <div class="step-header" :style="{ borderColor: stepColor(4) }">
          <div class="step-num-badge" :style="{ background: stepColor(4) }">5</div>
          <div>
            <div class="step-label">Paso 5</div>
            <div class="step-title">Instalar Herramientas de Desarrollo</div>
          </div>
          <div class="step-time-badge">
            <q-icon name="schedule" size="xs" /> {{ stepTime(4) }}
          </div>
        </div>

        <div class="devtools-grid q-mb-lg">
          <div class="dt-card" v-for="dt in devToolsInfo" :key="dt.name">
            <q-icon :name="dt.icon" size="md" :color="dt.color" class="q-mb-sm" />
            <div class="dt-name"><code>{{ dt.name }}</code></div>
            <div class="dt-desc">{{ dt.desc }}</div>
            <div class="dt-analogy">
              <q-icon name="compare_arrows" size="xs" color="amber" />
              {{ dt.analogy }}
            </div>
          </div>
        </div>

        <CodeBlock
          title="Instalar ros-dev-tools"
          lang="bash"
          :content="cmd.devTools"
          :copyable="true"
        />
      </section>

      <!-- ╔══════════════════════════════════════════════╗ -->
      <!-- ║    PASO 6: CONFIGURAR ENTORNO (.bashrc)      ║ -->
      <!-- ╚══════════════════════════════════════════════╝ -->
      <section class="page-section step-section" :ref="el => setStepRef(el, 5)">
        <div class="step-header" :style="{ borderColor: stepColor(5) }">
          <div class="step-num-badge" :style="{ background: stepColor(5) }">6</div>
          <div>
            <div class="step-label">Paso 6</div>
            <div class="step-title">Configurar el Entorno Automáticamente</div>
          </div>
          <div class="step-time-badge">
            <q-icon name="schedule" size="xs" /> {{ stepTime(5) }}
          </div>
        </div>

        <q-expansion-item
          class="why-expansion q-mb-md"
          icon="help_outline"
          label="¿Qué es el .bashrc y por qué lo modificamos?"
          expand-separator
          header-class="why-header"
        >
          <div class="why-body">
            <p>
              El archivo <code>~/.bashrc</code> es un <strong>script que se ejecuta automáticamente</strong>
              cada vez que abres una terminal. Es como una lista de tareas de preparación.
            </p>
            <p>
              Al agregar <code>source /opt/ros/jazzy/setup.bash</code> a ese archivo, le decimos al
              sistema: <em>"cada vez que abra una terminal, carga todas las herramientas de ROS 2"</em>.
              Sin esto, tendrías que escribir ese comando manualmente en cada sesión.
            </p>
          </div>
        </q-expansion-item>

        <SplitBlock>
          <template #left>
            <CodeBlock
              title="Agregar ROS 2 al .bashrc"
              lang="bash"
              :content="cmd.bashrc"
              :copyable="true"
            />
          </template>
          <template #right>
            <AlertBlock type="warning" title="Paso obligatorio tras ejecutar">
              Debes <strong>cerrar y abrir una nueva terminal</strong> (o ejecutar
              <code>source ~/.bashrc</code>) para que los cambios surtan efecto.
              ROS 2 no estará disponible en la terminal actual.
            </AlertBlock>

            <div class="expected-output q-mt-md">
              <div class="eo-label">
                <q-icon name="check_circle" color="positive" size="xs" />
                Verificar que funcionó:
              </div>
              <div class="eo-content">
                <div class="t-line"><span class="t-prompt">$</span> <span class="t-cmd">echo $ROS_DISTRO</span></div>
                <div class="t-line t-output t-highlight">jazzy</div>
              </div>
            </div>
          </template>
        </SplitBlock>
      </section>

      <!-- ╔══════════════════════════════════════════════╗ -->
      <!-- ║    PASO 7: CREAR WORKSPACE                   ║ -->
      <!-- ╚══════════════════════════════════════════════╝ -->
      <section class="page-section step-section" :ref="el => setStepRef(el, 6)">
        <div class="step-header" :style="{ borderColor: stepColor(6) }">
          <div class="step-num-badge" :style="{ background: stepColor(6) }">7</div>
          <div>
            <div class="step-label">Paso 7</div>
            <div class="step-title">Crear tu Workspace de Desarrollo</div>
          </div>
          <div class="step-time-badge">
            <q-icon name="schedule" size="xs" /> {{ stepTime(6) }}
          </div>
        </div>

        <q-expansion-item
          class="why-expansion q-mb-md"
          icon="help_outline"
          label="¿Qué es un Workspace en ROS 2?"
          expand-separator
          header-class="why-header"
        >
          <div class="why-body">
            <p>
              Un <strong>workspace</strong> es una carpeta organizada donde viven todos tus proyectos
              de ROS 2. Piénsala como el "directorio de trabajo" de un carpintero: el lugar donde
              están sus materiales, sus herramientas y sus proyectos en proceso.
            </p>
            <p>
              ROS 2 tiene una estructura de carpetas muy específica dentro del workspace.
              No es algo que inventes tú — es un estándar que todos siguen.
            </p>
          </div>
        </q-expansion-item>

        <SplitBlock>
          <template #left>
            <div class="ws-diagram">
              <div class="wsd-root">
                <q-icon name="folder" color="amber" size="md" />
                <span>ros2_ws/</span>
                <q-badge color="primary" class="q-ml-sm">Tu workspace</q-badge>
              </div>
              <div class="wsd-children">
                <div class="wsd-child" v-for="f in wsFolders" :key="f.name">
                  <div class="wsd-connector"></div>
                  <div class="wsd-node">
                    <q-icon :name="f.icon" :color="f.color" size="sm" />
                    <div class="wsd-text">
                      <span class="wsd-name">{{ f.name }}</span>
                      <span class="wsd-desc">{{ f.desc }}</span>
                    </div>
                    <q-badge v-if="f.badge" :color="f.badgeColor" outline dense>{{ f.badge }}</q-badge>
                  </div>
                </div>
              </div>
            </div>
          </template>

          <template #right>
            <CodeBlock
              title="Crear y compilar el workspace"
              lang="bash"
              :content="cmd.workspace"
              :copyable="true"
            />

            <AlertBlock type="info" title="Primera compilación" class="q-mt-md">
              La primera vez que ejecutas <code>colcon build</code> en un workspace vacío es normal
              que muestre <em>"0 packages finished"</em>. Eso es correcto — el workspace quedó
              inicializado y listo para recibir paquetes.
            </AlertBlock>
          </template>
        </SplitBlock>
      </section>

      <!-- ╔══════════════════════════════════════════════╗ -->
      <!-- ║    VERIFICACIÓN FINAL                        ║ -->
      <!-- ╚══════════════════════════════════════════════╝ -->
      <section class="page-section">
        <SectionTitle>Verificación Final — ¿Todo Funciona?</SectionTitle>
        <TextBlock>
          Ejecuta estas 3 pruebas en orden. Si todas pasan, tu instalación está
          <strong>100% operativa</strong>.
        </TextBlock>

        <div class="verify-grid q-mt-lg">
          <div class="verify-card" v-for="(test, i) in verifyTests" :key="i">
            <div class="vcard-header" :style="{ borderColor: test.color }">
              <div class="vcard-num" :style="{ background: test.color }">{{ i + 1 }}</div>
              <q-icon :name="test.icon" size="sm" :style="{ color: test.color }" />
              <div class="vcard-title">{{ test.title }}</div>
            </div>

            <div class="vcard-body">
              <div class="vcard-desc">{{ test.desc }}</div>

              <CodeBlock :lang="'bash'" :content="test.cmd" :copyable="true" class="q-mt-sm" />

              <div class="expected-output q-mt-sm">
                <div class="eo-label">
                  <q-icon name="check_circle" color="positive" size="xs" />
                  Resultado esperado:
                </div>
                <div class="eo-content" v-html="test.expected"></div>
              </div>
            </div>
          </div>
        </div>
      </section>

      <!-- ╔══════════════════════════════════════════════╗ -->
      <!-- ║    ERRORES COMUNES                           ║ -->
      <!-- ╚══════════════════════════════════════════════╝ -->
      <section class="page-section">
        <SectionTitle>Errores Comunes y Cómo Resolverlos</SectionTitle>
        <TextBlock>
          Si algo salió mal, no entres en pánico. El 90% de los errores de instalación tienen una
          solución directa. Busca tu error aquí:
        </TextBlock>

        <div class="trouble-accordion q-mt-lg">
          <q-expansion-item
            v-for="(err, i) in commonErrors"
            :key="i"
            class="trouble-item"
            :header-class="'trouble-header ' + err.level"
            expand-separator
            expand-icon="expand_more"
          >
            <template v-slot:header>
              <div class="trouble-h-content">
                <div class="trouble-badge" :class="err.level">
                  {{ err.level === 'error' ? '❌' : '⚠️' }}
                </div>
                <div class="trouble-h-text">
                  <div class="trouble-h-title">{{ err.title }}</div>
                  <div class="trouble-h-msg"><code>{{ err.msg }}</code></div>
                </div>
              </div>
            </template>

            <div class="trouble-body">
              <div class="trouble-cause">
                <strong>Causa:</strong> {{ err.cause }}
              </div>
              <div class="trouble-fix">
                <strong>Solución:</strong>
                <div v-html="err.fix" class="q-mt-xs"></div>
              </div>
              <CodeBlock v-if="err.cmd" lang="bash" :content="err.cmd" :copyable="true" class="q-mt-md" />
            </div>
          </q-expansion-item>
        </div>
      </section>

      <!-- ╔══════════════════════════════════════════════╗ -->
      <!-- ║    VIDEO TUTORIAL                            ║ -->
      <!-- ╚══════════════════════════════════════════════╝ -->
      <section class="page-section">
        <SectionTitle>Video Tutorial Complementario</SectionTitle>
        <TextBlock>
          Si prefieres ver el proceso completo en acción antes de ejecutar los comandos,
          este video muestra cada paso con comentarios:
        </TextBlock>

        <div class="video-shell q-mt-lg">
          <div class="video-aspect">
            <iframe
              src="https://www.youtube.com/embed/Romc22GgusU"
              title="Instalación ROS 2 Jazzy Jalisco"
              frameborder="0"
              allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture"
              allowfullscreen
              loading="lazy"
            />
          </div>
          <div class="video-meta">
            <q-icon name="play_circle" color="red-5" size="sm" />
            Instalación completa de ROS 2 Jazzy Jalisco en Ubuntu 24.04
          </div>
        </div>
      </section>

      <!-- ╔══════════════════════════════════════════════╗ -->
      <!-- ║    CTA FINAL                                 ║ -->
      <!-- ╚══════════════════════════════════════════════╝ -->
      <section class="page-section final-section">
        <div class="final-card">
          <div class="final-confetti">
            <div v-for="i in 8" :key="i" class="confetti-piece" :style="confettiStyle(i)"></div>
          </div>
          <div class="final-icon">
            <q-icon name="rocket_launch" size="4rem" color="primary" />
          </div>
          <h2 class="final-title">¡Instalación Completada!</h2>
          <p class="final-sub">
            Has configurado <strong>ROS 2 Jazzy</strong> desde cero. Tu entorno está listo para
            desarrollar robots profesionales. El siguiente paso es aprender los fundamentos del
            sistema operativo Linux que usarás todo el tiempo.
          </p>

          <div class="final-checklist q-mb-xl">
            <div class="fc-item" v-for="done in finalDoneList" :key="done">
              <q-icon name="check_circle" color="positive" size="sm" />
              <span>{{ done }}</span>
            </div>
          </div>

          <div class="final-actions">
            <q-btn
              color="primary"
              unelevated
              rounded
              size="lg"
              padding="16px 40px"
              to="/modulo-0/01navsistemaPage"
              icon-right="arrow_forward"
              label="Siguiente: Fundamentos de Linux"
              class="text-weight-bold"
            />
            <q-btn
              color="secondary"
              outline
              rounded
              size="md"
              padding="12px 28px"
              to="/"
              icon="home"
              label="Volver al inicio"
            />
          </div>
        </div>
      </section>

    </div><!-- /page-body -->
  </q-page>
</template>

<script setup lang="ts">
import { ref, computed, onMounted, type ComponentPublicInstance } from 'vue';
import TextBlock from 'components/content/TextBlock.vue';
import AlertBlock from 'components/content/AlertBlock.vue';
import CodeBlock from 'components/content/CodeBlock.vue';
import SectionTitle from 'components/content/SectionTitle.vue';
import SplitBlock from 'components/content/SplitBlock.vue';

// ─── Partículas hero ────────────────────────────────────────────────────
function particleStyle(i: number) {
  const colors = ['#38bdf8', '#818cf8', '#4ade80', '#f472b6', '#fb923c'];
  const color = colors[i % colors.length];
  return {
    left: `${(i * 8.33) % 100}%`,
    top: `${Math.sin(i * 1.3) * 40 + 50}%`,
    background: color,
    animationDelay: `${i * 0.4}s`,
    animationDuration: `${3 + (i % 3)}s`,
    width: `${4 + (i % 3) * 2}px`,
    height: `${4 + (i % 3) * 2}px`,
  };
}

function confettiStyle(i: number) {
  const colors = ['#38bdf8', '#818cf8', '#4ade80', '#f472b6', '#fb923c', '#facc15', '#34d399', '#a78bfa'];
  return {
    left: `${10 + (i - 1) * 12}%`,
    background: colors[i % colors.length],
    animationDelay: `${i * 0.15}s`,
  };
}

// ─── ¿Qué vamos a instalar? ─────────────────────────────────────────────
const whatWeInstall = [
  {
    icon: 'laptop_chromebook',
    color: '#f97316',
    title: 'Ubuntu 24.04',
    version: 'Noble Numbat · LTS',
    desc: 'El sistema operativo Linux donde todo correrá. Gratuito, seguro y ampliamente usado en robótica e industria.',
    analogy: 'Como el piso de un edificio: todo lo demás se construye sobre él.',
  },
  {
    icon: 'hub',
    color: '#38bdf8',
    title: 'ROS 2 Jazzy',
    version: 'Jazzy Jalisco · LTS 2029',
    desc: 'Robot Operating System 2. El framework que conecta todos los componentes de un robot: sensores, actuadores, algoritmos.',
    analogy: 'Como el sistema nervioso: coordina y comunica todas las partes.',
  },
  {
    icon: 'code',
    color: '#4ade80',
    title: 'Python 3.12',
    version: 'Incluido en Ubuntu 24.04',
    desc: 'El lenguaje de programación principal que usaremos. Ya viene preinstalado en Ubuntu — no necesitas instalarlo.',
    analogy: 'El idioma que usaremos para darle inteligencia al robot.',
  },
  {
    icon: 'build',
    color: '#818cf8',
    title: 'colcon + rosdep',
    version: 'ros-dev-tools',
    desc: 'Herramientas de compilación y gestión de dependencias. colcon construye tus paquetes, rosdep instala lo que necesitan.',
    analogy: 'Como un compilador + gestor de paquetes (similar a npm o pip).',
  },
];

// ─── Plataformas ─────────────────────────────────────────────────────────
const selectedPlatform = ref('native');
const platforms = [
  { id: 'native', icon: 'laptop_linux', color: 'orange', name: 'Ubuntu 24.04 Nativo', subtitle: 'Ya tengo Ubuntu instalado', recommended: true },
  { id: 'vm', icon: 'dns', color: 'blue', name: 'Máquina Virtual', subtitle: 'Tengo Windows o macOS', recommended: false },
  { id: 'wsl2', icon: 'window', color: 'cyan', name: 'WSL2 (Windows)', subtitle: 'Subsistema Linux en Windows', recommended: false },
];

const vmSteps = [
  'Descarga <strong>VirtualBox</strong> (gratis) desde virtualbox.org o <strong>VMware Workstation Player</strong>',
  'Descarga la imagen ISO de <strong>Ubuntu 24.04 LTS</strong> desde ubuntu.com (archivo ~6GB)',
  'Crea una nueva VM: asigna al menos <strong>4 núcleos de CPU, 8GB de RAM y 50GB de disco</strong>',
  'Instala Ubuntu desde la ISO. Durante la instalación selecciona <em>"Minimal installation"</em>',
  'Una vez instalado Ubuntu, <strong>instala Guest Additions</strong> para mejor rendimiento gráfico',
  'Continúa con el paso 1 de esta guía dentro de la VM',
];

// ─── Cómo abrir la terminal ──────────────────────────────────────────────
const terminalMethods = [
  { key: 'shortcut', icon: 'keyboard', title: 'Atajo de teclado', desc: 'La forma más rápida', shortcut: 'Ctrl + Alt + T' },
  { key: 'search', icon: 'search', title: 'Búsqueda del sistema', desc: 'Presiona la tecla Super (Windows) y escribe "Terminal"', shortcut: '' },
  { key: 'rightclick', icon: 'mouse', title: 'Click derecho en el escritorio', desc: 'En algunos temas de Ubuntu aparece "Open Terminal Here"', shortcut: '' },
];

// ─── Pre-flight checklist ────────────────────────────────────────────────
const preflightItems = ref([
  { label: 'Ubuntu 24.04 (Noble) instalado', verify: 'lsb_release -a', hint: 'Debe mostrar "24.04" y "noble"', checked: false },
  { label: 'Al menos 20GB de espacio libre en disco', verify: 'df -h', hint: 'Busca la fila de "/" y revisa la columna "Disponible"', checked: false },
  { label: 'Conexión a internet activa y estable', verify: 'ping -c 3 google.com', hint: 'Debes ver respuestas sin pérdida de paquetes', checked: false },
  { label: 'Permisos de administrador (sudo)', verify: 'sudo -v', hint: 'Te pedirá tu contraseña. Si la acepta, está bien', checked: false },
  { label: 'Virtualización habilitada (solo VMs)', verify: 'egrep -c "(vmx|svm)" /proc/cpuinfo', hint: 'Si el resultado es 0 en una VM, habilita virtualización en la BIOS', checked: false },
]);

const checkedCount = computed(() => preflightItems.value.filter(i => i.checked).length);
const preflightProgress = computed(() => Math.round((checkedCount.value / preflightItems.value.length) * 100));

// ─── Roadmap de pasos ────────────────────────────────────────────────────
const activeStep = ref(0);
const stepRefs = ref<HTMLElement[]>([]);

function setStepRef(el: Element | ComponentPublicInstance | null, idx: number) {
  if (el instanceof HTMLElement) stepRefs.value[idx] = el;
}

const installSteps = [
  { title: 'Verificar OS', icon: 'verified', color: '#38bdf8', time: '2 min' },
  { title: 'Configurar Locale', icon: 'language', color: '#818cf8', time: '3 min' },
  { title: 'Agregar Repos', icon: 'source', color: '#f472b6', time: '5 min' },
  { title: 'Instalar ROS 2', icon: 'download', color: '#4ade80', time: '15-25 min' },
  { title: 'Dev Tools', icon: 'build', color: '#fb923c', time: '3 min' },
  { title: 'Configurar .bashrc', icon: 'terminal', color: '#facc15', time: '2 min' },
  { title: 'Crear Workspace', icon: 'folder_open', color: '#34d399', time: '3 min' },
];

function scrollToStep(idx: number) {
  activeStep.value = idx;
  stepRefs.value[idx]?.scrollIntoView({ behavior: 'smooth', block: 'start' });
}

function stepColor(idx: number): string { return installSteps[idx]?.color ?? '#38bdf8'; }
function stepTime(idx: number): string  { return installSteps[idx]?.time  ?? ''; }

// ─── Selector de variante ────────────────────────────────────────────────
const selectedVariant = ref('desktop-full');
const variants = [
  {
    id: 'desktop-full',
    icon: 'monitor',
    color: 'primary',
    name: 'Desktop Full',
    pkg: 'ros-jazzy-desktop-full',
    size: '~3 GB',
    recommended: true,
    includes: ['ROS Core completo', 'RViz2 (visualización)', 'Gazebo simulador', 'Demos y ejemplos'],
  },
  {
    id: 'desktop',
    icon: 'desktop_windows',
    color: 'cyan',
    name: 'Desktop',
    pkg: 'ros-jazzy-desktop',
    size: '~800 MB',
    recommended: false,
    includes: ['ROS Core', 'RViz2 (visualización)', 'Demos y ejemplos'],
  },
  {
    id: 'base',
    icon: 'memory',
    color: 'grey',
    name: 'ROS Base',
    pkg: 'ros-jazzy-ros-base',
    size: '~300 MB',
    recommended: false,
    includes: ['ROS Core únicamente', 'Sin herramientas gráficas'],
  },
];

const installCommand = computed(() => {
  const pkgMap: Record<string, string> = {
    'desktop-full': 'ros-jazzy-desktop-full',
    desktop: 'ros-jazzy-desktop',
    base: 'ros-jazzy-ros-base',
  };
  const pkg = pkgMap[selectedVariant.value] ?? 'ros-jazzy-desktop-full';
  return `sudo apt update
sudo apt upgrade -y
sudo apt install ${pkg} -y`;
});

// ─── Dev tools info ──────────────────────────────────────────────────────
const devToolsInfo = [
  { icon: 'construction', color: 'orange', name: 'colcon', desc: 'Sistema de compilación para paquetes ROS 2. Construye tu código fuente en Python/C++.', analogy: 'Equivale a "make" o "cmake" en C++, o "python setup.py" en Python' },
  { icon: 'account_tree', color: 'cyan', name: 'rosdep', desc: 'Gestor de dependencias de ROS. Instala automáticamente lo que cada paquete necesita.', analogy: 'Como npm install o pip install, pero para el ecosistema ROS' },
  { icon: 'manage_search', color: 'purple', name: 'ros2cli', desc: 'La interfaz de línea de comandos de ROS 2. Todos los comandos ros2 que usarás.', analogy: 'Como el CLI de git: la herramienta con la que interactúas día a día' },
];

// ─── Workspace folders ───────────────────────────────────────────────────
const wsFolders = [
  { icon: 'folder_open', color: 'amber', name: 'src/', desc: 'Tu código fuente — aquí viven tus paquetes', badge: 'Tú trabajas aquí', badgeColor: 'primary' },
  { icon: 'folder', color: 'orange', name: 'build/', desc: 'Archivos intermedios de compilación', badge: 'Auto-generado', badgeColor: 'grey' },
  { icon: 'folder', color: 'green', name: 'install/', desc: 'Binarios listos para ejecutar', badge: 'Auto-generado', badgeColor: 'grey' },
  { icon: 'folder', color: 'blue-grey', name: 'log/', desc: 'Registros de compilación y errores', badge: 'Auto-generado', badgeColor: 'grey' },
];

// ─── Verificación tests ──────────────────────────────────────────────────
const verifyTests = [
  {
    icon: 'verified',
    color: '#38bdf8',
    title: 'Verificar versión de ROS 2',
    desc: 'Comprueba que ROS 2 Jazzy se instaló correctamente y está en el PATH.',
    cmd: 'ros2 --version',
    expected: '<span>ros2 cli version: <strong class="eo-highlight">jazzy</strong></span>',
  },
  {
    icon: 'chat',
    color: '#4ade80',
    title: 'Talker & Listener (comunicación básica)',
    desc: 'Abre DOS terminales. En la primera ejecuta el talker, en la segunda el listener. El listener debe recibir los mensajes.',
    cmd: `# Terminal 1 (Publicador):
ros2 run demo_nodes_cpp talker

# Terminal 2 (Suscriptor — en otra ventana):
ros2 run demo_nodes_py listener`,
    expected: `<span>Terminal 1 → <strong class="eo-highlight">[INFO] Publishing: 'Hello World: 1'</strong></span><br/><span>Terminal 2 → <strong class="eo-highlight">[INFO] I heard: 'Hello World: 1'</strong></span>`,
  },
  {
    icon: '3d_rotation',
    color: '#818cf8',
    title: 'Abrir RViz2 (visualización 3D)',
    desc: 'Lanza la interfaz gráfica de visualización de ROS 2. Solo aplica si instalaste Desktop o Desktop Full.',
    cmd: 'rviz2',
    expected: '<span>Debe abrirse la ventana gráfica de <strong class="eo-highlight">RViz2</strong> sin errores en la terminal</span>',
  },
];

// ─── Errores comunes ─────────────────────────────────────────────────────
const commonErrors = [
  {
    level: 'error',
    title: 'ROS 2 no se encuentra',
    msg: 'ros2: command not found',
    cause: 'El entorno de ROS 2 no se cargó en esta terminal.',
    fix: 'Ejecuta el comando de abajo <strong>en la terminal donde falló</strong>. Después agrega esto al .bashrc para que sea permanente.',
    cmd: 'source /opt/ros/jazzy/setup.bash',
  },
  {
    level: 'error',
    title: 'Paquete no encontrado',
    msg: 'Unable to locate package ros-jazzy-desktop',
    cause: 'Los repositorios de ROS 2 no están configurados correctamente.',
    fix: 'Repite el <strong>Paso 3</strong> completo y luego ejecuta <code>sudo apt update</code>. Verifica que la salida mencione <em>packages.ros.org</em>.',
    cmd: 'sudo apt update | grep ros',
  },
  {
    level: 'error',
    title: 'Dependencias no satisfechas',
    msg: 'Unmet dependencies / dpkg errors',
    cause: 'Conflictos entre paquetes de Ubuntu instalados previamente.',
    fix: 'Ejecuta el comando de reparación automática:',
    cmd: 'sudo apt --fix-broken install && sudo apt autoremove -y',
  },
  {
    level: 'warning',
    title: 'Error de llave GPG',
    msg: 'GPG error: NO_PUBKEY / Signatures were invalid',
    cause: 'La llave de cifrado del repositorio expiró o no se importó correctamente.',
    fix: 'Repite el paso 3 completo. Si persiste, importa la llave manualmente:',
    cmd: 'sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -',
  },
  {
    level: 'warning',
    title: 'RViz2 no abre (error de display)',
    msg: 'Could not connect to display / libGL error',
    cause: 'Problema con aceleración gráfica, común en máquinas virtuales sin 3D habilitado.',
    fix: 'En VirtualBox: <strong>Configuración → Pantalla → habilitar aceleración 3D</strong>. En VMware: instala VMware Tools. Como alternativa rápida:',
    cmd: 'LIBGL_ALWAYS_SOFTWARE=1 rviz2',
  },
  {
    level: 'error',
    title: 'Workspace no compiló',
    msg: 'colcon: command not found',
    cause: 'ros-dev-tools no se instaló o el entorno no está cargado.',
    fix: 'Verifica la instalación de dev tools y recarga el entorno:',
    cmd: 'sudo apt install ros-dev-tools -y && source /opt/ros/jazzy/setup.bash',
  },
  {
    level: 'warning',
    title: 'Descarga muy lenta',
    msg: 'Progreso < 50 KB/s durante más de 10 minutos',
    cause: 'El servidor de paquetes está saturado o el mirror es lento.',
    fix: 'Ve a <strong>Software & Updates → Ubuntu Software</strong> y cambia el servidor de descarga a uno más cercano geográficamente. Luego repite el <code>sudo apt install</code>.',
    cmd: '',
  },
  {
    level: 'error',
    title: 'Error de locale',
    msg: 'locale: Cannot set LC_ALL to default locale',
    cause: 'El locale en_US.UTF-8 no se generó correctamente.',
    fix: 'Regenera el locale y reconfigura el paquete:',
    cmd: 'sudo locale-gen en_US.UTF-8 && sudo dpkg-reconfigure locales',
  },
];

// ─── Final checklist ─────────────────────────────────────────────────────
const finalDoneList = [
  'Ubuntu 24.04 verificado y configurado',
  'Repositorios oficiales de ROS 2 agregados',
  'ROS 2 Jazzy Jalisco instalado',
  'Herramientas de desarrollo (colcon, rosdep) instaladas',
  'Entorno configurado en .bashrc',
  'Workspace ros2_ws creado y compilado',
];

// ─── Comandos ────────────────────────────────────────────────────────────
const repoMiniSteps = [
  'Habilitar repositorio <strong>Universe</strong> de Ubuntu',
  'Instalar herramientas de gestión (<code>curl</code>, <code>software-properties-common</code>)',
  'Descargar e instalar el <strong>paquete de configuración</strong> de ROS 2',
  'Actualizar el índice de paquetes de apt',
];

const cmd = {
  osCheck: `lsb_release -a
# Debes ver "Ubuntu 24.04" y "noble"`,

  locale: `locale  # Ver configuración actual

sudo apt update && sudo apt install locales -y
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # Verificar: debe mostrar UTF-8`,

  repos: '# 1. Habilitar repositorio Universe\n'
    + 'sudo apt install software-properties-common -y\n'
    + 'sudo add-apt-repository universe\n\n'
    + '# 2. Instalar curl\n'
    + 'sudo apt update && sudo apt install curl -y\n\n'
    + '# 3. Obtener la versión más reciente del paquete de configuración\n'
    + 'export ROS_APT_SOURCE_VERSION=$(curl -s \\\n'
    + '  https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \\\n'
    + '  | grep -F "tag_name" | awk -F\'"\' \'{print $4}\')\n\n'
    + '# 4. Descargar el paquete de configuración\n'
    + 'curl -L -o /tmp/ros2-apt-source.deb \\\n'
    + '  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/\\\n'
    + '${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.\\\n'
    + '$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"\n\n'
    + '# 5. Instalar\n'
    + 'sudo dpkg -i /tmp/ros2-apt-source.deb\n\n'
    + '# 6. Actualizar índice de paquetes\n'
    + 'sudo apt update',

  devTools: `sudo apt update
sudo apt install ros-dev-tools -y

# Verificar que colcon está disponible:
colcon --version`,

  bashrc: '# Agregar ROS 2 al inicio automático de la terminal\n'
    + 'echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc\n\n'
    + '# Agregar el workspace (lo crearemos en el paso 7)\n'
    + 'echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc\n\n'
    + '# Recargar la configuración\n'
    + 'source ~/.bashrc\n\n'
    + '# Verificar: debe mostrar "jazzy"\n'
    + 'echo $ROS_DISTRO',

  workspace: `# Crear la estructura del workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Compilar el workspace vacío (inicialización)
colcon build

# Verificar: debe mostrar la estructura de carpetas
ls ~/ros2_ws/`,
};

onMounted(() => {
  const observer = new IntersectionObserver(
    (entries) => {
      entries.forEach((entry) => {
        if (entry.isIntersecting) {
          const idx = stepRefs.value.indexOf(entry.target as HTMLElement);
          if (idx !== -1) activeStep.value = idx;
        }
      });
    },
    { threshold: 0.3 },
  );
  stepRefs.value.forEach((el) => { observer.observe(el); });
});
</script>

<style scoped>
/* ══════════════════════════════════════════════════════
   BASE
══════════════════════════════════════════════════════ */
.installation-page {
  background: var(--bg-page);
  min-height: 100vh;
}

.page-body {
  max-width: 1100px;
  margin: 0 auto;
  padding: 0 24px 80px 24px;
}

.page-section {
  margin-bottom: 5rem;
}

/* ══════════════════════════════════════════════════════
   HERO
══════════════════════════════════════════════════════ */
.install-hero {
  position: relative;
  padding: 6rem 2rem 5rem 2rem;
  text-align: center;
  background:
    radial-gradient(ellipse at top left, rgba(56, 189, 248, 0.15) 0%, transparent 55%),
    radial-gradient(ellipse at bottom right, rgba(129, 140, 248, 0.12) 0%, transparent 55%),
    var(--bg-surface);
  border-bottom: 1px solid var(--border-subtle);
  overflow: hidden;
  margin-bottom: 4rem;
}

.hero-inner {
  position: relative;
  z-index: 2;
  max-width: 820px;
  margin: 0 auto;
}

.hero-badge {
  display: inline-flex;
  align-items: center;
  gap: 6px;
  background: var(--bg-code);
  color: var(--text-code);
  border: 1px solid var(--border-hover);
  padding: 6px 14px;
  border-radius: 999px;
  font-size: 0.8rem;
  font-weight: 700;
  letter-spacing: 0.05em;
  text-transform: uppercase;
  margin-bottom: 1.5rem;
}

.hero-h1 {
  font-size: 3.2rem;
  font-weight: 900;
  line-height: 1.15;
  color: var(--text-primary);
  letter-spacing: -0.03em;
  margin: 0 0 1.25rem 0;
}

.text-gradient-hero {
  background: linear-gradient(135deg, #06b6d4 0%, #8b5cf6 40%, #3b82f6 100%);
  -webkit-background-clip: text;
  -webkit-text-fill-color: transparent;
  background-clip: text;
}

.hero-sub {
  font-size: 1.2rem;
  color: var(--text-secondary);
  line-height: 1.65;
  max-width: 640px;
  margin: 0 auto 2rem auto;
}

.hero-stats {
  display: flex;
  flex-wrap: wrap;
  gap: 10px;
  justify-content: center;
}

.stat-chip {
  display: inline-flex;
  align-items: center;
  gap: 6px;
  background: var(--bg-surface-solid);
  border: 1px solid var(--border-medium);
  color: var(--text-secondary);
  padding: 8px 16px;
  border-radius: 999px;
  font-size: 0.9rem;
  font-weight: 500;
}

/* Partículas flotantes */
.hero-particles {
  position: absolute;
  inset: 0;
  pointer-events: none;
  z-index: 1;
}
.particle {
  position: absolute;
  border-radius: 50%;
  opacity: 0.4;
  animation: particle-float 4s ease-in-out infinite alternate;
}
@keyframes particle-float {
  from { transform: translateY(0) scale(1); opacity: 0.3; }
  to   { transform: translateY(-20px) scale(1.3); opacity: 0.6; }
}

/* Animaciones hero */
.animate-fade-in        { animation: fadeIn 0.8s ease-out; }
.animate-slide-up       { animation: slideUp 0.8s ease-out 0.15s both; }
.animate-slide-up-delay { animation: slideUp 0.8s ease-out 0.3s both; }
.animate-fade-in-delay  { animation: fadeIn 0.8s ease-out 0.5s both; }

@keyframes fadeIn {
  from { opacity: 0; }
  to   { opacity: 1; }
}
@keyframes slideUp {
  from { opacity: 0; transform: translateY(24px); }
  to   { opacity: 1; transform: translateY(0); }
}

/* ══════════════════════════════════════════════════════
   ¿QUÉ VAMOS A INSTALAR?
══════════════════════════════════════════════════════ */
.explain-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(240px, 1fr));
  gap: 20px;
}

.explain-card {
  background: var(--bg-surface-solid);
  border: 1px solid var(--border-subtle);
  border-radius: 16px;
  padding: 24px;
  display: flex;
  flex-direction: column;
  gap: 12px;
  transition: all 0.25s ease;
}

.explain-card:hover {
  transform: translateY(-4px);
  border-color: var(--border-hover);
  box-shadow: 0 12px 32px var(--shadow-md);
}

.explain-icon {
  width: 56px;
  height: 56px;
  border-radius: 14px;
  border: 1px solid;
  display: flex;
  align-items: center;
  justify-content: center;
}

.explain-title {
  font-size: 1.05rem;
  font-weight: 700;
  color: var(--text-primary);
}

.explain-version {
  font-size: 0.78rem;
  font-weight: 600;
  color: var(--text-muted);
  font-family: 'Fira Code', monospace;
}

.explain-desc {
  font-size: 0.9rem;
  color: var(--text-secondary);
  line-height: 1.55;
}

.explain-analogy {
  display: flex;
  align-items: center;
  gap: 6px;
  font-size: 0.82rem;
  color: var(--text-muted);
  font-style: italic;
  border-top: 1px solid var(--border-subtle);
  padding-top: 10px;
  margin-top: 4px;
}

/* ══════════════════════════════════════════════════════
   SELECTOR DE PLATAFORMA
══════════════════════════════════════════════════════ */
.platform-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
  gap: 16px;
  max-width: 700px;
}

.platform-card {
  background: var(--bg-surface-solid);
  border: 2px solid var(--border-subtle);
  border-radius: 16px;
  padding: 24px 20px;
  text-align: center;
  cursor: pointer;
  transition: all 0.25s ease;
  position: relative;
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 8px;
}

.platform-card:hover {
  border-color: var(--border-hover);
  background: var(--bg-surface-hover);
  transform: translateY(-2px);
}

.platform-selected {
  border-color: var(--border-hover) !important;
  background: var(--bg-code) !important;
}

.platform-name {
  font-weight: 700;
  color: var(--text-primary);
  font-size: 0.95rem;
}

.platform-sub {
  font-size: 0.8rem;
  color: var(--text-muted);
}

.platform-badge, .platform-selected-badge {
  position: absolute;
  top: -8px;
  right: 12px;
}

/* VMs guide */
.vm-guide { max-width: 800px; }

.vm-steps {
  background: var(--bg-surface-solid);
  border: 1px solid var(--border-subtle);
  border-radius: 12px;
  padding: 20px;
  display: flex;
  flex-direction: column;
  gap: 14px;
}

.vm-step {
  display: flex;
  align-items: flex-start;
  gap: 14px;
}

.vm-step-num {
  width: 28px;
  height: 28px;
  flex-shrink: 0;
  border-radius: 50%;
  background: linear-gradient(135deg, #38bdf8, #6366f1);
  color: #fff;
  font-weight: 700;
  font-size: 0.85rem;
  display: flex;
  align-items: center;
  justify-content: center;
}

.vm-step-text {
  color: var(--text-secondary);
  font-size: 0.95rem;
  line-height: 1.5;
  padding-top: 3px;
}

/* ══════════════════════════════════════════════════════
   CÓMO ABRIR LA TERMINAL
══════════════════════════════════════════════════════ */
.terminal-guide {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(220px, 1fr));
  gap: 16px;
}

.tg-method {
  background: var(--bg-surface-solid);
  border: 1px solid var(--border-subtle);
  border-radius: 14px;
  padding: 20px;
  display: flex;
  flex-direction: column;
  align-items: flex-start;
  gap: 10px;
  transition: all 0.2s ease;
}

.tg-method:hover {
  border-color: var(--border-hover);
}

.tg-icon {
  width: 44px;
  height: 44px;
  background: var(--bg-code);
  border-radius: 10px;
  display: flex;
  align-items: center;
  justify-content: center;
}

.tg-title {
  font-weight: 700;
  color: var(--text-primary);
  font-size: 1rem;
}

.tg-desc {
  color: var(--text-secondary);
  font-size: 0.88rem;
  line-height: 1.5;
}

.tg-shortcut {
  display: inline-flex;
  align-items: center;
  gap: 6px;
  background: var(--bg-surface);
  border: 1px solid var(--border-medium);
  border-radius: 6px;
  padding: 4px 10px;
  font-size: 0.85rem;
  font-weight: 700;
  color: var(--text-code);
  font-family: 'Fira Code', monospace;
}

/* Terminal mock */
.terminal-mock {
  max-width: 680px;
  border-radius: 14px;
  overflow: hidden;
  border: 1px solid var(--border-medium);
  box-shadow: 0 16px 40px var(--shadow-md);
  font-family: 'Fira Code', 'Monaco', monospace;
}

.terminal-mock-bar {
  background: #1e2435;
  padding: 10px 16px;
  display: flex;
  align-items: center;
  gap: 8px;
}
[data-theme='light'] .terminal-mock-bar { background: #e2e8f0; }

.dot {
  width: 12px;
  height: 12px;
  border-radius: 50%;
  flex-shrink: 0;
}
.dot.red    { background: #ef4444; }
.dot.yellow { background: #f59e0b; }
.dot.green  { background: #22c55e; }

.terminal-mock-title {
  flex: 1;
  text-align: center;
  font-size: 0.78rem;
  color: #64748b;
}

.terminal-mock-body {
  background: #0f1629;
  padding: 16px 20px;
  font-size: 0.88rem;
  line-height: 1.8;
}
[data-theme='light'] .terminal-mock-body { background: #1e293b; }

.t-line { display: flex; flex-wrap: wrap; align-items: center; gap: 6px; }
.t-prompt { color: #4ade80; font-weight: 700; }
.t-cmd    { color: #f8fafc; }
.t-output { color: #94a3b8; padding-left: 0; }
.t-highlight { color: #38bdf8 !important; font-weight: 600; }
.t-cursor { color: #38bdf8; animation: blink 1s step-end infinite; }

@keyframes blink { 50% { opacity: 0; } }

.mock-caption {
  text-align: center;
  color: var(--text-muted);
  font-size: 0.82rem;
  margin-top: 10px;
}

/* ══════════════════════════════════════════════════════
   PRE-FLIGHT CHECKLIST
══════════════════════════════════════════════════════ */
.preflight-wrapper {
  max-width: 820px;
  margin: 0 auto;
  background: var(--bg-surface-solid);
  border: 1px solid var(--border-subtle);
  border-radius: 20px;
  overflow: hidden;
}

.preflight-items {
  display: flex;
  flex-direction: column;
}

.preflight-item {
  display: flex;
  align-items: flex-start;
  gap: 16px;
  padding: 18px 24px;
  border-bottom: 1px solid var(--border-subtle);
  cursor: pointer;
  transition: all 0.2s ease;
}

.preflight-item:hover {
  background: var(--bg-surface-hover);
}

.preflight-item:last-child { border-bottom: none; }

.item-checked {
  background: rgba(74, 222, 128, 0.06) !important;
}

.pf-check { flex-shrink: 0; margin-top: 2px; }

.pf-label {
  font-weight: 600;
  color: var(--text-primary);
  font-size: 1rem;
  margin-bottom: 4px;
  transition: all 0.2s;
}

.pf-done {
  text-decoration: line-through;
  color: var(--text-muted) !important;
}

.pf-detail {
  font-size: 0.82rem;
  color: var(--text-code);
  font-family: 'Fira Code', monospace;
  margin-bottom: 4px;
  display: flex;
  align-items: center;
}

.pf-hint {
  font-size: 0.82rem;
  color: var(--text-muted);
}

.pf-status { margin-left: auto; flex-shrink: 0; }

.preflight-footer {
  padding: 20px 24px;
  border-top: 1px solid var(--border-subtle);
  background: var(--bg-surface);
}

.pf-progress-label {
  display: flex;
  justify-content: space-between;
  color: var(--text-secondary);
  font-size: 0.9rem;
  font-weight: 600;
  margin-bottom: 8px;
}

.pf-pct { color: var(--text-primary); font-weight: 700; }

.pf-ready {
  display: flex;
  align-items: center;
  gap: 8px;
  margin-top: 14px;
  color: #4ade80;
  font-weight: 700;
  font-size: 0.95rem;
}

/* ══════════════════════════════════════════════════════
   ROADMAP DE PASOS
══════════════════════════════════════════════════════ */
.roadmap-steps {
  display: flex;
  flex-wrap: wrap;
  gap: 0;
  background: var(--bg-surface-solid);
  border: 1px solid var(--border-subtle);
  border-radius: 16px;
  overflow: hidden;
}

.rmap-step {
  flex: 1;
  min-width: 100px;
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 8px;
  padding: 20px 12px;
  cursor: pointer;
  position: relative;
  transition: all 0.2s ease;
  border-right: 1px solid var(--border-subtle);
}

.rmap-step:last-child { border-right: none; }

.rmap-step:hover { background: var(--bg-surface-hover); }

.rmap-active { background: var(--bg-code) !important; }

.rmap-num {
  width: 40px;
  height: 40px;
  border-radius: 50%;
  border: 2px solid;
  display: flex;
  align-items: center;
  justify-content: center;
  flex-shrink: 0;
}

.rmap-title {
  font-size: 0.78rem;
  font-weight: 700;
  color: var(--text-primary);
  text-align: center;
  line-height: 1.3;
}

.rmap-time {
  font-size: 0.72rem;
  color: var(--text-muted);
  text-align: center;
}

/* ══════════════════════════════════════════════════════
   SECCIONES DE PASOS
══════════════════════════════════════════════════════ */
.step-section {
  scroll-margin-top: 80px;
}

.step-header {
  display: flex;
  align-items: center;
  gap: 16px;
  padding: 20px 24px;
  background: var(--bg-surface-solid);
  border: 1px solid var(--border-subtle);
  border-left: 4px solid;
  border-radius: 14px;
  margin-bottom: 24px;
}

.step-num-badge {
  width: 36px;
  height: 36px;
  border-radius: 50%;
  color: #fff;
  font-weight: 900;
  font-size: 1rem;
  display: flex;
  align-items: center;
  justify-content: center;
  flex-shrink: 0;
}

.step-label {
  font-size: 0.75rem;
  font-weight: 700;
  text-transform: uppercase;
  letter-spacing: 0.08em;
  color: var(--text-muted);
}

.step-title {
  font-size: 1.35rem;
  font-weight: 800;
  color: var(--text-primary);
  line-height: 1.2;
}

.step-time-badge {
  margin-left: auto;
  display: flex;
  align-items: center;
  gap: 5px;
  font-size: 0.82rem;
  color: var(--text-muted);
  background: var(--bg-surface);
  border: 1px solid var(--border-subtle);
  padding: 4px 10px;
  border-radius: 999px;
  white-space: nowrap;
  flex-shrink: 0;
}

/* Why expansion */
.why-expansion {
  background: var(--bg-surface-solid) !important;
  border: 1px solid var(--border-subtle) !important;
  border-radius: 10px !important;
  overflow: hidden;
}

:deep(.why-header) {
  color: var(--text-secondary) !important;
  font-weight: 600;
  font-size: 0.92rem;
}

.why-body {
  padding: 16px 20px;
  color: var(--text-secondary);
  font-size: 0.9rem;
  line-height: 1.6;
  background: var(--bg-surface);
}

.why-body p { margin: 0 0 10px 0; }
.why-body p:last-child { margin-bottom: 0; }

/* ══════════════════════════════════════════════════════
   TABLA DE COMPATIBILIDAD
══════════════════════════════════════════════════════ */
.compat-table {
  background: var(--bg-surface-solid);
  border: 1px solid var(--border-subtle);
  border-radius: 12px;
  overflow: hidden;
  font-size: 0.9rem;
}

.ct-row {
  display: grid;
  grid-template-columns: 1.5fr 1fr 1fr;
  padding: 10px 16px;
  border-bottom: 1px solid var(--border-subtle);
  color: var(--text-secondary);
  align-items: center;
}
.ct-row:last-child { border-bottom: none; }

.ct-header {
  background: var(--bg-surface);
  font-weight: 700;
  color: var(--text-primary);
  font-size: 0.8rem;
  text-transform: uppercase;
  letter-spacing: 0.06em;
}

.ct-ok   { border-left: 4px solid #38bdf8; background: rgba(56,189,248,0.06); font-weight: 600; }
.ct-warn { border-left: 4px solid #f59e0b; opacity: 0.8; }
.ct-bad  { border-left: 4px solid #ef4444; opacity: 0.5; }

.ct-badge {
  display: inline-block;
  font-size: 0.8rem;
  font-weight: 600;
}

/* ══════════════════════════════════════════════════════
   RESULTADO ESPERADO
══════════════════════════════════════════════════════ */
.expected-output {
  background: var(--bg-surface-solid);
  border: 1px solid var(--border-subtle);
  border-left: 3px solid #4ade80;
  border-radius: 10px;
  padding: 14px 18px;
  font-size: 0.88rem;
}

.eo-label {
  display: flex;
  align-items: center;
  gap: 6px;
  font-weight: 700;
  color: var(--text-secondary);
  margin-bottom: 8px;
  font-size: 0.82rem;
  text-transform: uppercase;
  letter-spacing: 0.05em;
}

.eo-content {
  font-family: 'Fira Code', monospace;
  color: var(--text-secondary);
  font-size: 0.88rem;
  line-height: 1.7;
}

:deep(.eo-highlight) {
  color: #4ade80;
  font-weight: 700;
}

/* ══════════════════════════════════════════════════════
   MINI STEPS (REPOS)
══════════════════════════════════════════════════════ */
.mini-steps {
  display: flex;
  flex-direction: column;
  gap: 10px;
  max-width: 700px;
}

.mini-step {
  display: flex;
  align-items: center;
  gap: 14px;
  background: var(--bg-surface-solid);
  border: 1px solid var(--border-subtle);
  border-radius: 10px;
  padding: 12px 18px;
}

.ms-num {
  width: 26px;
  height: 26px;
  border-radius: 50%;
  background: linear-gradient(135deg, #f472b6, #818cf8);
  color: #fff;
  font-weight: 700;
  font-size: 0.82rem;
  display: flex;
  align-items: center;
  justify-content: center;
  flex-shrink: 0;
}

.ms-text {
  color: var(--text-secondary);
  font-size: 0.92rem;
  line-height: 1.4;
}

/* ══════════════════════════════════════════════════════
   SELECTOR DE VARIANTE
══════════════════════════════════════════════════════ */
.variant-selector {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
  gap: 16px;
}

.variant-card {
  background: var(--bg-surface-solid);
  border: 2px solid var(--border-subtle);
  border-radius: 16px;
  padding: 24px 20px;
  cursor: pointer;
  transition: all 0.25s ease;
  position: relative;
  display: flex;
  flex-direction: column;
  gap: 10px;
}

.variant-card:hover {
  border-color: var(--border-hover);
  transform: translateY(-2px);
}

.variant-selected {
  border-color: var(--border-hover) !important;
  background: var(--bg-code) !important;
}

.vc-top {
  display: flex;
  align-items: flex-start;
  justify-content: space-between;
}

.vc-name {
  font-weight: 800;
  color: var(--text-primary);
  font-size: 1.05rem;
}

.vc-pkg {
  font-size: 0.78rem;
  color: var(--text-code);
  font-family: 'Fira Code', monospace;
  background: var(--bg-code);
  padding: 3px 8px;
  border-radius: 6px;
}

.vc-size {
  font-size: 0.82rem;
  color: var(--text-muted);
  display: flex;
  align-items: center;
  gap: 4px;
}

.vc-includes {
  list-style: none;
  padding: 0;
  margin: 0;
  display: flex;
  flex-direction: column;
  gap: 4px;
}

.vc-includes li {
  display: flex;
  align-items: center;
  gap: 6px;
  font-size: 0.83rem;
  color: var(--text-secondary);
}

.vc-rec {
  position: absolute;
  top: -8px;
  right: 12px;
}

/* ══════════════════════════════════════════════════════
   INSTALL INFO BOX
══════════════════════════════════════════════════════ */
.install-info-box {
  background: var(--bg-surface-solid);
  border: 1px solid var(--border-subtle);
  border-radius: 14px;
  overflow: hidden;
}

.iib-item {
  display: flex;
  align-items: flex-start;
  gap: 14px;
  padding: 16px 20px;
  border-bottom: 1px solid var(--border-subtle);
}
.iib-item:last-child { border-bottom: none; }

.iib-title {
  font-weight: 700;
  color: var(--text-primary);
  font-size: 0.92rem;
  margin-bottom: 3px;
}

.iib-desc {
  font-size: 0.85rem;
  color: var(--text-secondary);
  line-height: 1.4;
}

/* ══════════════════════════════════════════════════════
   DEV TOOLS GRID
══════════════════════════════════════════════════════ */
.devtools-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(230px, 1fr));
  gap: 16px;
}

.dt-card {
  background: var(--bg-surface-solid);
  border: 1px solid var(--border-subtle);
  border-radius: 14px;
  padding: 24px 20px;
  display: flex;
  flex-direction: column;
  gap: 8px;
  transition: all 0.2s ease;
}

.dt-card:hover {
  border-color: var(--border-hover);
  transform: translateY(-3px);
}

.dt-name {
  font-weight: 800;
  color: var(--text-primary);
  font-size: 1.05rem;
}

.dt-desc {
  font-size: 0.88rem;
  color: var(--text-secondary);
  line-height: 1.5;
}

.dt-analogy {
  display: flex;
  align-items: flex-start;
  gap: 6px;
  font-size: 0.8rem;
  color: var(--text-muted);
  font-style: italic;
  border-top: 1px solid var(--border-subtle);
  padding-top: 10px;
}

/* ══════════════════════════════════════════════════════
   WORKSPACE DIAGRAM
══════════════════════════════════════════════════════ */
.ws-diagram {
  background: var(--bg-surface-solid);
  border: 1px solid var(--border-subtle);
  border-radius: 16px;
  padding: 24px;
  font-family: 'Fira Code', monospace;
}

.wsd-root {
  display: flex;
  align-items: center;
  gap: 10px;
  font-size: 1.1rem;
  font-weight: 700;
  color: var(--text-primary);
  margin-bottom: 16px;
}

.wsd-children {
  display: flex;
  flex-direction: column;
  gap: 8px;
  margin-left: 20px;
  border-left: 2px solid var(--border-medium);
  padding-left: 16px;
}

.wsd-child {
  display: flex;
  align-items: flex-start;
  gap: 10px;
}

.wsd-connector {
  display: none;
}

.wsd-node {
  flex: 1;
  display: flex;
  align-items: center;
  gap: 10px;
  background: var(--bg-surface);
  border: 1px solid var(--border-subtle);
  border-radius: 8px;
  padding: 10px 14px;
}

.wsd-text {
  flex: 1;
  display: flex;
  flex-direction: column;
  gap: 2px;
}

.wsd-name {
  color: var(--text-primary);
  font-weight: 600;
  font-size: 0.9rem;
}

.wsd-desc {
  color: var(--text-muted);
  font-family: 'Inter', sans-serif;
  font-size: 0.78rem;
}

/* ══════════════════════════════════════════════════════
   VERIFICACIÓN FINAL
══════════════════════════════════════════════════════ */
.verify-grid {
  display: flex;
  flex-direction: column;
  gap: 24px;
}

.verify-card {
  background: var(--bg-surface-solid);
  border: 1px solid var(--border-subtle);
  border-radius: 16px;
  overflow: hidden;
}

.vcard-header {
  display: flex;
  align-items: center;
  gap: 12px;
  padding: 16px 20px;
  border-bottom: 1px solid var(--border-subtle);
  border-top: 3px solid;
  background: var(--bg-surface);
}

.vcard-num {
  width: 32px;
  height: 32px;
  border-radius: 50%;
  color: #fff;
  font-weight: 800;
  font-size: 0.9rem;
  display: flex;
  align-items: center;
  justify-content: center;
  flex-shrink: 0;
}

.vcard-title {
  font-weight: 700;
  color: var(--text-primary);
  font-size: 1.02rem;
}

.vcard-body {
  padding: 20px;
}

.vcard-desc {
  color: var(--text-secondary);
  font-size: 0.9rem;
  line-height: 1.55;
  margin-bottom: 12px;
}

/* ══════════════════════════════════════════════════════
   TROUBLESHOOTING ACCORDION
══════════════════════════════════════════════════════ */
.trouble-accordion {
  display: flex;
  flex-direction: column;
  gap: 10px;
}

.trouble-item {
  background: var(--bg-surface-solid) !important;
  border: 1px solid var(--border-subtle) !important;
  border-radius: 12px !important;
  overflow: hidden;
}

:deep(.trouble-header) {
  min-height: 64px !important;
}

:deep(.trouble-header.error) {
  border-left: 4px solid #ef4444;
}
:deep(.trouble-header.warning) {
  border-left: 4px solid #f59e0b;
}

.trouble-h-content {
  display: flex;
  align-items: center;
  gap: 14px;
  flex: 1;
  padding: 4px 0;
}

.trouble-badge {
  font-size: 1.2rem;
  flex-shrink: 0;
}

.trouble-h-title {
  font-weight: 700;
  color: var(--text-primary);
  font-size: 0.95rem;
  margin-bottom: 3px;
}

.trouble-h-msg {
  font-size: 0.8rem;
  color: var(--text-code);
  font-family: 'Fira Code', monospace;
}

.trouble-body {
  padding: 20px;
  border-top: 1px solid var(--border-subtle);
  background: var(--bg-surface);
}

.trouble-cause {
  color: var(--text-secondary);
  font-size: 0.9rem;
  margin-bottom: 12px;
  line-height: 1.5;
}

.trouble-fix {
  color: var(--text-secondary);
  font-size: 0.9rem;
  line-height: 1.5;
}

.trouble-fix :deep(code) {
  background: var(--bg-code);
  color: var(--text-code);
  padding: 1px 5px;
  border-radius: 4px;
  font-family: 'Fira Code', monospace;
  font-size: 0.85em;
}

/* ══════════════════════════════════════════════════════
   VIDEO
══════════════════════════════════════════════════════ */
.video-shell {
  background: var(--bg-surface-solid);
  border: 1px solid var(--border-medium);
  border-radius: 20px;
  overflow: hidden;
  max-width: 900px;
  margin: 0 auto;
}

.video-aspect {
  position: relative;
  aspect-ratio: 16/9;
  background: #000;
}

.video-aspect iframe {
  position: absolute;
  inset: 0;
  width: 100%;
  height: 100%;
}

.video-meta {
  padding: 14px 20px;
  display: flex;
  align-items: center;
  gap: 8px;
  font-size: 0.88rem;
  color: var(--text-muted);
  background: var(--bg-surface);
  border-top: 1px solid var(--border-subtle);
}

/* ══════════════════════════════════════════════════════
   CTA FINAL
══════════════════════════════════════════════════════ */
.final-section { text-align: center; }

.final-card {
  background: var(--bg-surface-solid);
  border: 1px solid var(--border-medium);
  border-radius: 28px;
  padding: 4rem 3rem;
  max-width: 760px;
  margin: 0 auto;
  position: relative;
  overflow: hidden;
}

.final-confetti {
  position: absolute;
  inset: 0;
  pointer-events: none;
}

.confetti-piece {
  position: absolute;
  top: -10px;
  width: 10px;
  height: 10px;
  border-radius: 2px;
  opacity: 0.6;
  animation: confetti-fall 3s ease-in infinite;
}

@keyframes confetti-fall {
  0%   { transform: translateY(-20px) rotate(0deg); opacity: 0.7; }
  100% { transform: translateY(200px) rotate(720deg); opacity: 0; }
}

.final-icon {
  margin-bottom: 1.5rem;
  animation: bounce-icon 2s ease-in-out infinite;
}
@keyframes bounce-icon {
  0%, 100% { transform: translateY(0); }
  50%       { transform: translateY(-10px); }
}

.final-title {
  font-size: 2rem;
  font-weight: 900;
  color: var(--text-primary);
  margin: 0 0 1rem 0;
}

.final-sub {
  color: var(--text-secondary);
  font-size: 1.05rem;
  line-height: 1.65;
  max-width: 560px;
  margin: 0 auto 2rem auto;
}

.final-checklist {
  display: inline-flex;
  flex-direction: column;
  gap: 10px;
  text-align: left;
}

.fc-item {
  display: flex;
  align-items: center;
  gap: 10px;
  font-size: 0.95rem;
  color: var(--text-secondary);
}

.final-actions {
  display: flex;
  gap: 16px;
  justify-content: center;
  flex-wrap: wrap;
}

/* Transition helpers */
.fade-enter-active, .fade-leave-active { transition: opacity 0.25s ease; }
.fade-enter-from, .fade-leave-to       { opacity: 0; }

/* ══════════════════════════════════════════════════════
   RESPONSIVE
══════════════════════════════════════════════════════ */
@media (max-width: 768px) {
  .hero-h1  { font-size: 2.2rem; }
  .hero-sub { font-size: 1.05rem; }

  .roadmap-steps {
    display: grid;
    grid-template-columns: repeat(4, 1fr);
  }
  .rmap-step { border-right: none; border-bottom: 1px solid var(--border-subtle); }

  .explain-grid    { grid-template-columns: 1fr; }
  .variant-selector { grid-template-columns: 1fr; }
  .devtools-grid   { grid-template-columns: 1fr; }
  .platform-grid   { grid-template-columns: 1fr; }

  .step-header    { flex-wrap: wrap; gap: 10px; }
  .step-time-badge { order: 3; }

  .final-card     { padding: 2.5rem 1.5rem; }
  .final-title    { font-size: 1.5rem; }
  .final-actions  { flex-direction: column; align-items: center; }

  .page-body { padding: 0 16px 60px 16px; }
}

@media (max-width: 480px) {
  .hero-h1    { font-size: 1.85rem; }
  .hero-stats { flex-direction: column; align-items: center; }
  .roadmap-steps { grid-template-columns: repeat(2, 1fr); }
}
</style>
