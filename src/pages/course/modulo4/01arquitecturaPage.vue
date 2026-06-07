<template>
  <LessonContainer>

    <!-- INTRO + FACT PILLS -->
    <div class="section-group">
      <TextBlock>
        ROS 2 implementa un <strong>grafo computacional distribuido</strong>: cada nodo es un proceso
        independiente que se comunica mediante el middleware DDS. Esta arquitectura desacoplada
        permite aislar fallos, escalar horizontalmente y mezclar C++, Python y Rust en el mismo robot.
      </TextBlock>
      <div class="fact-pills q-mt-lg">
        <div v-for="f in facts" :key="f.label" class="fact-pill">
          <span class="fp-icon">{{ f.icon }}</span>
          <span class="fp-text">{{ f.label }}</span>
        </div>
      </div>
    </div>

    <!-- ══ 01 GRAFO COMPUTACIONAL ══ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge green">01</span>
        El Grafo Computacional de ROS 2
      </SectionTitle>
      <TextBlock>
        El <strong>grafo computacional</strong> es la red de nodos que componen tu robot. Cada nodo
        publica o suscribe a topics, ofrece servicios, o ejecuta acciones. Puedes visualizarlo
        en tiempo real con <code>rqt_graph</code>:
      </TextBlock>

      <!-- Animated computation graph (SVG) -->
      <div class="cg-wrapper q-mt-xl">
        <div class="cg-title">Grafo de un robot TurtleBot3 típico</div>
        <svg class="cg-svg" viewBox="0 0 820 300" xmlns="http://www.w3.org/2000/svg">
          <!-- Column labels -->
          <text x="110" y="22" text-anchor="middle" fill="#4ade80" font-size="11" font-weight="700" font-family="inherit">SENSORES</text>
          <text x="410" y="22" text-anchor="middle" fill="#60a5fa" font-size="11" font-weight="700" font-family="inherit">PROCESAMIENTO</text>
          <text x="710" y="22" text-anchor="middle" fill="#f87171" font-size="11" font-weight="700" font-family="inherit">ACTUACIÓN</text>

          <!-- SENSOR NODES -->
          <rect x="35" y="50" width="150" height="58" rx="12" fill="#4ade8012" stroke="#4ade80" stroke-width="1.5"/>
          <text x="110" y="75" text-anchor="middle" fill="#4ade80" font-size="11" font-weight="700" font-family="Fira Code, monospace">/camera_node</text>
          <text x="110" y="93" text-anchor="middle" fill="#94a3b8" font-size="10" font-family="inherit">pub: /image_raw</text>

          <rect x="35" y="192" width="150" height="58" rx="12" fill="#4ade8012" stroke="#4ade80" stroke-width="1.5"/>
          <text x="110" y="217" text-anchor="middle" fill="#4ade80" font-size="11" font-weight="700" font-family="Fira Code, monospace">/lidar_node</text>
          <text x="110" y="235" text-anchor="middle" fill="#94a3b8" font-size="10" font-family="inherit">pub: /scan</text>

          <!-- PROCESSING NODES -->
          <rect x="335" y="50" width="150" height="58" rx="12" fill="#60a5fa12" stroke="#60a5fa" stroke-width="1.5"/>
          <text x="410" y="75" text-anchor="middle" fill="#60a5fa" font-size="11" font-weight="700" font-family="Fira Code, monospace">/nav2_node</text>
          <text x="410" y="93" text-anchor="middle" fill="#94a3b8" font-size="10" font-family="inherit">pub: /cmd_vel</text>

          <rect x="335" y="192" width="150" height="58" rx="12" fill="#fbbf2412" stroke="#fbbf24" stroke-width="1.5"/>
          <text x="410" y="217" text-anchor="middle" fill="#fbbf24" font-size="11" font-weight="700" font-family="Fira Code, monospace">/slam_node</text>
          <text x="410" y="235" text-anchor="middle" fill="#94a3b8" font-size="10" font-family="inherit">pub: /map</text>

          <!-- ACTUATOR NODES -->
          <rect x="635" y="50" width="150" height="58" rx="12" fill="#f8717112" stroke="#f87171" stroke-width="1.5"/>
          <text x="710" y="75" text-anchor="middle" fill="#f87171" font-size="11" font-weight="700" font-family="Fira Code, monospace">/control_node</text>
          <text x="710" y="93" text-anchor="middle" fill="#94a3b8" font-size="10" font-family="inherit">sub: /cmd_vel</text>

          <rect x="635" y="192" width="150" height="58" rx="12" fill="#c084fc12" stroke="#c084fc" stroke-width="1.5"/>
          <text x="710" y="217" text-anchor="middle" fill="#c084fc" font-size="11" font-weight="700" font-family="Fira Code, monospace">/rviz_node</text>
          <text x="710" y="235" text-anchor="middle" fill="#94a3b8" font-size="10" font-family="inherit">sub: /map</text>

          <!-- STATIC CONNECTION LINES (background) -->
          <line x1="185" y1="79" x2="335" y2="79" stroke="#4ade8030" stroke-width="2" stroke-dasharray="6,4"/>
          <line x1="185" y1="221" x2="335" y2="221" stroke="#4ade8030" stroke-width="2" stroke-dasharray="6,4"/>
          <line x1="185" y1="221" x2="335" y2="79" stroke="#60a5fa20" stroke-width="1.5" stroke-dasharray="4,4"/>
          <line x1="485" y1="79" x2="635" y2="79" stroke="#60a5fa30" stroke-width="2" stroke-dasharray="6,4"/>
          <line x1="485" y1="221" x2="635" y2="221" stroke="#fbbf2430" stroke-width="2" stroke-dasharray="6,4"/>
          <line x1="410" y1="108" x2="410" y2="192" stroke="#fbbf2430" stroke-width="1.5" stroke-dasharray="4,4"/>

          <!-- ANIMATED FLOW LINES -->
          <line x1="185" y1="79" x2="335" y2="79" stroke="#4ade80" stroke-width="2" stroke-dasharray="12,88" class="cg-flow cg-f1"/>
          <line x1="185" y1="221" x2="335" y2="221" stroke="#4ade80" stroke-width="2" stroke-dasharray="12,88" class="cg-flow cg-f2"/>
          <line x1="485" y1="79" x2="635" y2="79" stroke="#60a5fa" stroke-width="2" stroke-dasharray="12,88" class="cg-flow cg-f3"/>
          <line x1="485" y1="221" x2="635" y2="221" stroke="#fbbf24" stroke-width="2" stroke-dasharray="12,88" class="cg-flow cg-f4"/>
          <line x1="410" y1="108" x2="410" y2="192" stroke="#fbbf24" stroke-width="1.5" stroke-dasharray="8,72" class="cg-flow cg-f5"/>

          <!-- TOPIC LABELS -->
          <rect x="214" y="62" width="72" height="16" rx="4" fill="#0d111799"/>
          <text x="250" y="74" text-anchor="middle" fill="#4ade80" font-size="9" font-family="Fira Code, monospace">/image_raw</text>
          <rect x="213" y="204" width="50" height="16" rx="4" fill="#0d111799"/>
          <text x="238" y="216" text-anchor="middle" fill="#4ade80" font-size="9" font-family="Fira Code, monospace">/scan</text>
          <rect x="516" y="62" width="68" height="16" rx="4" fill="#0d111799"/>
          <text x="550" y="74" text-anchor="middle" fill="#60a5fa" font-size="9" font-family="Fira Code, monospace">/cmd_vel</text>
          <rect x="526" y="204" width="44" height="16" rx="4" fill="#0d111799"/>
          <text x="548" y="216" text-anchor="middle" fill="#fbbf24" font-size="9" font-family="Fira Code, monospace">/map</text>
        </svg>
        <div class="cg-legend">
          <div class="cgl-item" :style="{ '--cgl-c': '#4ade80' }"><div class="cgl-dot"></div>Sensores</div>
          <div class="cgl-item" :style="{ '--cgl-c': '#60a5fa' }"><div class="cgl-dot"></div>Procesamiento</div>
          <div class="cgl-item" :style="{ '--cgl-c': '#fbbf24' }"><div class="cgl-dot"></div>Cartografía</div>
          <div class="cgl-item" :style="{ '--cgl-c': '#f87171' }"><div class="cgl-dot"></div>Control</div>
          <div class="cgl-item" :style="{ '--cgl-c': '#94a3b8' }"><div class="cgl-dot"></div>Topics (─ ─ ─)</div>
        </div>
      </div>

      <div class="cmd-showcase q-mt-xl">
        <div v-for="cs in graphCmds" :key="cs.cmd" class="cs-item" :style="{ '--cs-color': cs.color }">
          <div class="csi-header">
            <q-icon :name="cs.icon" size="16px" :style="{ color: cs.color }"/>
            <code class="csi-cmd">{{ cs.cmd }}</code>
          </div>
          <div class="csi-desc">{{ cs.desc }}</div>
        </div>
      </div>
    </div>

    <!-- ══ 02 MONO vs DIST ══ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge amber">02</span>
        Arquitectura Monolítica vs. Distribuida
      </SectionTitle>
      <TextBlock>
        El diseño de ROS 2 resuelve los problemas fundamentales del desarrollo robótico monolítico.
        La diferencia no es solo técnica — define cómo el equipo desarrolla, prueba y depura:
      </TextBlock>

      <div class="arch-grid q-mt-lg">
        <!-- Monolithic -->
        <div class="arch-card" :style="{ '--ac-color': '#f87171' }">
          <div class="acc-header">
            <div class="acc-icon" :style="{ background: '#f8717118', color: '#f87171' }">
              <q-icon name="warning" size="22px"/>
            </div>
            <div class="acc-title">Arquitectura Monolítica</div>
          </div>
          <div class="mono-block">
            <div v-for="m in monoModules" :key="m" class="mono-module">{{ m }}</div>
            <div class="mono-coupling">Acoplamiento fuerte — todo en un proceso</div>
          </div>
          <div class="acc-points">
            <div v-for="p in monoProblems" :key="p" class="acc-point negative">
              <q-icon name="close" size="14px" style="color:#f87171;flex-shrink:0"/>
              <span>{{ p }}</span>
            </div>
          </div>
        </div>

        <!-- Distributed -->
        <div class="arch-card" :style="{ '--ac-color': '#22d3ee' }">
          <div class="acc-header">
            <div class="acc-icon" :style="{ background: '#22d3ee18', color: '#22d3ee' }">
              <q-icon name="check_circle" size="22px"/>
            </div>
            <div class="acc-title">Arquitectura Distribuida (ROS 2)</div>
          </div>
          <div class="dist-graph">
            <div v-for="n in distNodes" :key="n.label" class="dist-node"
              :style="{ '--dn-color': n.color }">
              <q-icon :name="n.icon" size="16px" :style="{ color: n.color }"/>
              <span>{{ n.label }}</span>
            </div>
          </div>
          <div class="acc-points">
            <div v-for="b in distBenefits" :key="b" class="acc-point positive">
              <q-icon name="check" size="14px" style="color:#4ade80;flex-shrink:0"/>
              <span>{{ b }}</span>
            </div>
          </div>
        </div>
      </div>

      <div class="insight-box q-mt-lg" :style="{ '--ib-color': '#fbbf24' }">
        <div class="ib-header">
          <q-icon name="lightbulb" size="18px" style="color:#fbbf24"/>
          <strong>Component Containers — Lo mejor de ambos mundos</strong>
        </div>
        <p class="ib-desc">
          En sistemas avanzados, múltiples nodos pueden ejecutarse en un <strong>solo proceso</strong>
          mediante Composition. Se combina el desacoplamiento lógico de la arquitectura distribuida
          con la eficiencia de memoria del monolito. La comunicación intra-proceso evita
          serialización y copia de datos.
        </p>
      </div>
    </div>

    <!-- ══ 03 DDS MIDDLEWARE ══ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge cyan">03</span>
        DDS — El Middleware de Comunicación
      </SectionTitle>
      <TextBlock>
        ROS 2 usa <strong>DDS (Data Distribution Service)</strong> como capa de comunicación.
        DDS es un estándar industrial (OMG) que implementa publish-subscribe sobre UDP/TCP con
        QoS configurable. No es específico de robótica — se usa en aviónica, finanzas y defensa:
      </TextBlock>

      <!-- Animated DDS Layers -->
      <div class="dds-stack q-mt-xl">
        <div v-for="(layer, idx) in ddsLayers" :key="layer.title" class="dds-layer"
          :style="{ '--dl-color': layer.color, '--dl-delay': idx * 0.1 + 's' }">
          <div class="dll-left">
            <div class="dll-icon">
              <q-icon :name="layer.icon" size="22px" :style="{ color: layer.color }"/>
            </div>
            <div class="dll-info">
              <div class="dll-title" :style="{ color: layer.color }">{{ layer.title }}</div>
              <div class="dll-subtitle">{{ layer.subtitle }}</div>
            </div>
          </div>
          <div class="dll-right">
            <code v-for="tag in layer.tags" :key="tag" class="dll-tag"
              :style="{ color: layer.color, background: layer.color + '12', borderColor: layer.color + '30' }">
              {{ tag }}
            </code>
          </div>
          <div v-if="idx < ddsLayers.length - 1" class="dll-arrow">
            <div class="dlla-packet" :style="{ '--pack-c': layer.color }"></div>
            <q-icon name="arrow_downward" size="14px" style="color:var(--text-muted)"/>
          </div>
        </div>
      </div>

      <!-- Discovery phases -->
      <SectionTitle class="q-mt-xl">Protocolo de Descubrimiento Automático</SectionTitle>
      <TextBlock>
        Los nodos se encuentran entre sí sin configuración centralizada. El proceso usa dos
        protocolos: descubrimiento de participantes (SPDP) y de endpoints (SEDP):
      </TextBlock>

      <div class="discovery-grid q-mt-lg">
        <div v-for="(phase, idx) in discoveryPhases" :key="phase.title" class="disc-card"
          :style="{ '--dp-color': phase.color }">
          <div class="dpc-number" :style="{ background: phase.color, color: '#0d1117' }">{{ idx + 1 }}</div>
          <div class="dpc-body">
            <div class="dpc-title" :style="{ color: phase.color }">{{ phase.title }}</div>
            <p class="dpc-desc">{{ phase.desc }}</p>
            <div v-if="phase.warning" class="dpc-warn">
              <q-icon name="warning" size="14px" style="color:#fbbf24"/>
              {{ phase.warning }}
            </div>
            <CodeBlock :hide-header="true" lang="bash" :content="phase.code" :copyable="true"/>
          </div>
        </div>
      </div>
    </div>

    <!-- ══ 04 QoS PROFILES ══ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge purple">04</span>
        QoS Profiles — Control de Calidad de Servicio
      </SectionTitle>
      <TextBlock>
        Los <strong>QoS Profiles</strong> controlan cómo se comporta la comunicación. Son la
        diferencia entre perder datos de sensores (aceptable) y perder un comando de control
        (inaceptable). Publisher y Subscriber deben tener QoS compatibles:
      </TextBlock>

      <div class="qos-cards q-mt-lg">
        <div v-for="q in qosProfiles" :key="q.policy" class="qos-card"
          :style="{ '--qos-color': q.color }">
          <div class="qosc-header">
            <q-icon :name="q.icon" size="18px" :style="{ color: q.color }"/>
            <code class="qosc-policy">{{ q.policy }}</code>
          </div>
          <div class="qosc-options">
            <span v-for="opt in q.options" :key="opt" class="qosc-opt">{{ opt }}</span>
          </div>
          <div class="qosc-use">{{ q.use }}</div>
        </div>
      </div>

      <div class="compat-box q-mt-xl">
        <div class="cb-title">
          <q-icon name="compare_arrows" size="18px" color="warning"/>
          Compatibilidad QoS — La regla más importante
        </div>
        <div class="compat-grid">
          <div v-for="c in compatRules" :key="c.label" class="compat-item">
            <div class="ci-indicator">
              <q-icon :name="c.ok ? 'check_circle' : 'cancel'" size="16px"
                :style="{ color: c.ok ? '#4ade80' : '#f87171' }"/>
            </div>
            <code class="ci-pub">{{ c.pub }}</code>
            <q-icon name="arrow_forward" size="12px" style="color:var(--text-muted)"/>
            <code class="ci-sub">{{ c.sub }}</code>
            <span class="ci-label">{{ c.label }}</span>
          </div>
        </div>
      </div>

      <CodeBlock title="Configuración QoS por escenario" lang="cpp"
        :content="qosCode" :copyable="true" class="q-mt-xl"/>
    </div>

    <!-- ══ 05 LIFECYCLE ══ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge red">05</span>
        Ciclo de Vida del Nodo — Managed Nodes
      </SectionTitle>
      <TextBlock>
        Los <strong>Lifecycle Nodes</strong> tienen estados bien definidos con transiciones
        controladas. Permiten que el sistema de gestión inicie, configure y finalice nodos
        de forma determinista — esencial para robots en producción:
      </TextBlock>

      <!-- Lifecycle state machine -->
      <div class="lc-machine q-mt-xl">
        <div class="lcm-track">
          <div v-for="(state, idx) in lifecycleStates" :key="state.name" class="lcm-item">
            <div class="lcm-state" :style="{ '--lcs-color': state.color }"
              :class="{ 'lcm-active': state.active }">
              <q-icon :name="state.icon" size="20px" :style="{ color: state.color }"/>
              <div class="lcms-name">{{ state.name }}</div>
              <div class="lcms-sub">{{ state.sub }}</div>
            </div>
            <div v-if="idx < lifecycleStates.length - 1" class="lcm-transition">
              <div class="lcmt-fn" :style="{ color: lifecycleStates[idx]?.color }">
                {{ lifecycleStates[idx]?.transition }}
              </div>
              <q-icon name="arrow_forward" size="16px" style="color:var(--text-muted)"/>
            </div>
          </div>
        </div>
        <div class="lcm-back-arrows q-mt-md">
          <div class="lcm-back" :style="{ '--lb-color': '#fbbf24' }">
            cleanup() — vuelve a Unconfigured
          </div>
          <div class="lcm-back" :style="{ '--lb-color': '#f87171' }">
            shutdown() — en cualquier estado → Finalized
          </div>
          <div class="lcm-back" :style="{ '--lb-color': '#f87171' }">
            error → ErrorProcessing → cleanup()
          </div>
        </div>
      </div>

      <CodeBlock title="Implementar Lifecycle Node en C++" lang="cpp"
        :content="lifecycleCode" :copyable="true" class="q-mt-xl"/>
    </div>

    <!-- ══ 06 EXECUTORS ══ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge amber">06</span>
        Executors — Cómo ROS 2 Procesa Callbacks
      </SectionTitle>
      <TextBlock>
        El <strong>Executor</strong> es el bucle de eventos de ROS 2. Decide cuándo y en qué
        hilo se ejecutan los callbacks (subscriptores, timers, servicios). Elegir el executor
        correcto impacta directamente la latencia y el throughput:
      </TextBlock>

      <div class="exec-grid q-mt-lg">
        <div v-for="ex in executors" :key="ex.name" class="exec-card"
          :style="{ '--ex-color': ex.color }">
          <div class="exc-header">
            <div class="exc-icon" :style="{ background: ex.color + '18' }">
              <q-icon :name="ex.icon" size="24px" :style="{ color: ex.color }"/>
            </div>
            <div>
              <code class="exc-name">{{ ex.name }}</code>
              <div class="exc-badge" :style="{ background: ex.color + '15', color: ex.color }">
                {{ ex.badge }}
              </div>
            </div>
          </div>
          <p class="exc-desc">{{ ex.desc }}</p>
          <div class="exc-queue">
            <div class="excq-label">Cola de callbacks:</div>
            <div class="excq-track" :style="{ '--eq-color': ex.color }">
              <div v-for="(cb, i) in ex.callbacks" :key="i" class="excq-item"
                :class="{ 'excq-active': cb.active }"
                :style="{ '--eqi-c': cb.color, animationDelay: i * 0.4 + 's' }">
                {{ cb.label }}
              </div>
            </div>
          </div>
          <CodeBlock :hide-header="true" lang="cpp" :content="ex.code" :copyable="true"/>
        </div>
      </div>

      <div class="cbgroup-box q-mt-xl">
        <div class="cbg-title">
          <q-icon name="category" size="18px" color="primary"/>
          Callback Groups — Control de Concurrencia
        </div>
        <div class="cbg-grid">
          <div v-for="cg in cbGroups" :key="cg.name" class="cbg-card"
            :style="{ '--cbg-color': cg.color }">
            <div class="cbgc-name" :style="{ color: cg.color }">{{ cg.name }}</div>
            <div class="cbgc-desc">{{ cg.desc }}</div>
          </div>
        </div>
      </div>
    </div>

    <!-- ══ 07 DOMAIN ID y NAMESPACES ══ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge green">07</span>
        Domain ID y Namespaces — Organización y Aislamiento
      </SectionTitle>
      <TextBlock>
        <strong>Domain ID</strong> aísla grupos de nodos en la misma red — cada robot puede tener
        su propio dominio. <strong>Namespaces</strong> organizan jerárquicamente los nombres de
        nodos y topics dentro de un robot:
      </TextBlock>

      <div class="domain-visual q-mt-lg">
        <div v-for="domain in domains" :key="domain.id" class="domain-group"
          :style="{ '--dom-color': domain.color }">
          <div class="domg-header">
            <q-icon name="lan" size="16px" :style="{ color: domain.color }"/>
            Domain {{ domain.id }} — {{ domain.name }}
          </div>
          <div class="domg-nodes">
            <div v-for="n in domain.nodes" :key="n" class="domg-node">{{ n }}</div>
          </div>
        </div>
        <div class="domain-block">
          <q-icon name="block" size="32px" style="color:#f87171"/>
          <div class="dombl-text">Aislados</div>
        </div>
      </div>

      <div class="ns-visual q-mt-xl">
        <div class="nsv-title">Namespaces — Jerarquía de nombres</div>
        <div class="nsv-tree">
          <div v-for="ns in namespaceTree" :key="ns.path" class="nst-item"
            :style="{ paddingLeft: ns.depth * 20 + 'px', '--nst-color': ns.color }">
            <div class="nsti-line"></div>
            <code class="nsti-path" :style="{ color: ns.color }">{{ ns.path }}</code>
            <span class="nsti-type">{{ ns.type }}</span>
          </div>
        </div>
      </div>

      <CodeBlock title="Domain ID y Namespaces en práctica" lang="bash"
        :content="domainCode" :copyable="true" class="q-mt-xl"/>
    </div>

    <!-- ══ 08 RENDIMIENTO ══ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge cyan">08</span>
        Rendimiento — Latencia y Throughput por Transporte
      </SectionTitle>
      <TextBlock>
        El tipo de transporte tiene el mayor impacto en la latencia. La elección correcta
        puede reducir la latencia de <strong>8.5 ms a 8 μs</strong> — un factor de 1000×:
      </TextBlock>

      <!-- Animated performance bars -->
      <div class="perf-bars q-mt-xl">
        <div v-for="pb in perfBars" :key="pb.mode" class="pb-row">
          <div class="pbr-label">{{ pb.mode }}</div>
          <div class="pbr-track">
            <div class="pbr-fill" :style="{ '--pb-color': pb.color, '--pb-width': pb.pct + '%', animationDelay: pb.delay }"></div>
          </div>
          <div class="pbr-value" :style="{ color: pb.color }">{{ pb.latency }}</div>
        </div>
      </div>

      <div class="perf-cards q-mt-xl">
        <div v-for="pc in perfCards" :key="pc.title" class="perf-card"
          :style="{ '--pc-color': pc.color }">
          <div class="pcc-header">
            <q-icon :name="pc.icon" size="22px" :style="{ color: pc.color }"/>
            <span class="pcc-title">{{ pc.title }}</span>
          </div>
          <div class="pcc-metrics">
            <div v-for="m in pc.metrics" :key="m.label" class="pccm-row">
              <span class="pccm-label">{{ m.label }}</span>
              <span class="pccm-val" :style="{ color: m.good ? '#4ade80' : m.bad ? '#f87171' : '#fbbf24' }">
                {{ m.value }}
              </span>
            </div>
          </div>
          <p class="pcc-desc">{{ pc.desc }}</p>
          <CodeBlock :hide-header="true" :lang="pc.lang || 'cpp'" :content="pc.code" :copyable="true"/>
        </div>
      </div>

      <div class="q-my-xl">
        <RmwVisualizer />
      </div>

      <!-- Benchmark table -->
      <div class="bench-table q-mt-lg">
        <div class="bt-header">
          <div class="bt-cell">Tamaño</div>
          <div class="bt-cell" :style="{ color: '#4ade80' }">Intra-Process</div>
          <div class="bt-cell" :style="{ color: '#60a5fa' }">Shared Memory</div>
          <div class="bt-cell" :style="{ color: '#f87171' }">UDP (red)</div>
        </div>
        <div v-for="row in benchRows" :key="row.size" class="bt-row">
          <div class="bt-cell bt-cell-hd">{{ row.size }}</div>
          <div class="bt-cell" :style="{ color: '#4ade80' }">{{ row.intra }}</div>
          <div class="bt-cell" :style="{ color: '#60a5fa' }">{{ row.shm }}</div>
          <div class="bt-cell" :style="{ color: '#f87171' }">{{ row.udp }}</div>
        </div>
      </div>
      <div class="bench-note">
        Benchmarks: Intel i7-10700K, Ubuntu 24.04, ROS 2 Jazzy, FastDDS 3.x
      </div>
    </div>

    <!-- ══ ERRORES COMUNES ══ -->
    <div class="section-group">
      <SectionTitle>Errores Comunes de Arquitectura</SectionTitle>
      <div class="error-list q-mt-lg">
        <div v-for="(err, i) in commonErrors" :key="i" class="error-item"
          :style="{ '--err-color': err.color }">
          <div class="err-header" @click="err.open = !err.open">
            <div class="err-left">
              <div class="err-num" :style="{ background: err.color + '18', color: err.color }">{{ i + 1 }}</div>
              <div>
                <div class="err-type" :style="{ color: err.color }">{{ err.type }}</div>
                <div class="err-summary">{{ err.summary }}</div>
              </div>
            </div>
            <q-icon :name="err.open ? 'expand_less' : 'expand_more'"
              size="20px" style="color:var(--text-muted);flex-shrink:0"/>
          </div>
          <div v-show="err.open" class="err-body">
            <div class="err-cause">
              <q-icon name="search" size="14px" class="q-mr-xs"/>
              <strong>Situación:</strong> {{ err.cause }}
            </div>
            <CodeBlock :hide-header="true" lang="bash" :content="err.code"/>
            <div class="err-fix">
              <q-icon name="build" size="14px" class="q-mr-xs" color="positive"/>
              <strong>Solución:</strong> {{ err.fix }}
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- ══ RETO ══ -->
    <div class="section-group">
      <SectionTitle>Reto — Diagnosticar el Grafo del Robot</SectionTitle>
      <div class="challenge-box">
        <div class="challenge-header">
          <div class="challenge-icon">
            <q-icon name="emoji_events" size="28px" color="warning"/>
          </div>
          <div>
            <div class="challenge-title">Analiza y optimiza el grafo de comunicación</div>
            <div class="challenge-subtitle">Usa CLI de ROS 2 para inspeccionar nodos, topics y QoS</div>
          </div>
          <div class="challenge-badge">45 min</div>
        </div>
        <div class="challenge-steps q-mt-md">
          <div class="cs-title">Pasos:</div>
          <div class="cs-list">
            <div v-for="step in challengeSteps" :key="step.num" class="cs-item">
              <div class="cs-num" :style="{ background: step.color }">{{ step.num }}</div>
              <div class="cs-text">{{ step.text }}</div>
            </div>
          </div>
        </div>
        <CodeBlock title="Comandos del reto" lang="bash"
          :content="challengeCode" :copyable="true" class="q-mt-md"/>
        <q-expansion-item icon="lightbulb" label="Ver pistas"
          header-class="answer-header" class="q-mt-md">
          <div class="answer-body">
            <div v-for="h in challengeHints" :key="h" class="answer-row">
              <q-icon name="chevron_right" size="14px" style="color:#4ade80"/>
              {{ h }}
            </div>
          </div>
        </q-expansion-item>
      </div>
    </div>

    <!-- ══ VIDEO ══ -->
    <div class="section-group">
      <SectionTitle>Video Complementario</SectionTitle>
      <TextBlock>Arquitectura ROS 2 y DDS en profundidad:</TextBlock>
      <div class="video-card q-mt-md">
        <div class="video-wrapper">
          <iframe src="https://www.youtube.com/embed/Romc22GgusU"
            title="ROS 2 Architecture and DDS" frameborder="0"
            allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture"
            allowfullscreen></iframe>
        </div>
        <div class="video-caption">
          <q-icon name="info" size="16px" color="info" class="q-mr-sm"/>
          Video en revisión — será reemplazado con contenido específico del curso.
        </div>
      </div>
    </div>

    <!-- ══ RESUMEN ══ -->
    <div class="section-group q-mb-xl">
      <SectionTitle>Resumen de Arquitectura ROS 2</SectionTitle>
      <div class="summary-grid q-mt-lg">
        <div v-for="s in summaryItems" :key="s.term" class="summary-card"
          :style="{ '--sc-color': s.color }">
          <code class="sc-term">{{ s.term }}</code>
          <div class="sc-desc">{{ s.desc }}</div>
          <div class="sc-note">{{ s.note }}</div>
        </div>
      </div>
    </div>

    <!-- ══ CTA ══ -->
    <div class="section-group q-mt-xl">
      <div class="final-cta">
        <div class="fca-icon">
          <q-icon name="folder_open" size="40px" color="primary"/>
        </div>
        <h2 class="fca-title">¡Arquitectura comprendida!</h2>
        <p class="fca-sub">
          Entiendes el grafo computacional, DDS, QoS, lifecycle nodes y executors.
          El siguiente paso: crear tu propio workspace y escribir tu primer nodo en C++ y Python.
        </p>
        <div class="fca-actions">
          <q-btn color="primary" unelevated rounded size="lg" padding="14px 40px"
            to="/modulo-4/02workspacePage"
            icon="arrow_forward" label="Workspace y Paquetes"
            class="text-weight-bold"/>
        </div>
      </div>
    </div>

  </LessonContainer>
</template>

<script setup lang="ts">
import { reactive } from 'vue';
import LessonContainer from 'components/content/LessonContainer.vue';
import TextBlock from 'components/content/TextBlock.vue';
import CodeBlock from 'components/content/CodeBlock.vue';
import SectionTitle from 'components/content/SectionTitle.vue';
import RmwVisualizer from 'components/content/interactive/RmwVisualizer.vue';

// ═══════════════════════════════════════════════════
// CODE CONSTANTS
// ═══════════════════════════════════════════════════

const qosCode = [
  '// ── Sensor data (alta frecuencia, pérdida tolerada) ──',
  'auto sensor_qos = rclcpp::SensorDataQoS();',
  '// BEST_EFFORT | VOLATILE | KEEP_LAST(5)',
  '',
  '// ── Comandos de control (sin pérdida) ──',
  'auto ctrl_qos = rclcpp::QoS(10)',
  '  .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)',
  '  .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE)',
  '  .deadline(std::chrono::milliseconds(100));',
  '',
  '// ── Parámetros (persistentes, late-joiners) ──',
  'auto param_qos = rclcpp::ParametersQoS();',
  '// RELIABLE | TRANSIENT_LOCAL | KEEP_ALL',
  '',
  '// ── Verificar QoS de un topic en tiempo real ──',
  '// ros2 topic info /cmd_vel --verbose',
].join('\n');

const lifecycleCode = [
  '#include "rclcpp_lifecycle/lifecycle_node.hpp"',
  '',
  'class MotorDriver : public rclcpp_lifecycle::LifecycleNode {',
  'public:',
  '  MotorDriver() : LifecycleNode("motor_driver") {}',
  '',
  '  // 1. Unconfigured → Inactive',
  '  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {',
  '    motor_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);',
  '    return CallbackReturn::SUCCESS;',
  '  }',
  '',
  '  // 2. Inactive → Active',
  '  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override {',
  '    motor_pub_->on_activate();',
  '    RCLCPP_INFO(get_logger(), "Motor driver activated");',
  '    return CallbackReturn::SUCCESS;',
  '  }',
  '',
  '  // 3. Active → Inactive',
  '  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override {',
  '    motor_pub_->on_deactivate();',
  '    return CallbackReturn::SUCCESS;',
  '  }',
  '',
  '  // 4. Inactive → Unconfigured',
  '  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override {',
  '    motor_pub_.reset();',
  '    return CallbackReturn::SUCCESS;',
  '  }',
  '};',
].join('\n');

const domainCode = [
  '# Configurar Domain ID para tu robot',
  'export ROS_DOMAIN_ID=42',
  '',
  '# Verificar que estás en el dominio correcto',
  'echo $ROS_DOMAIN_ID',
  '',
  '# Lanzar nodo con namespace (robot1)',
  'ros2 run turtlebot3_bringup robot --ros-args --remap __ns:=/robot1',
  '',
  '# El topic pasa de /cmd_vel a /robot1/cmd_vel',
  'ros2 topic list | grep robot1',
  '# /robot1/cmd_vel',
  '# /robot1/odom',
  '',
  '# Remapping de nombre de nodo y topic',
  'ros2 run demo_nodes_cpp talker --ros-args \\',
  '  --remap __node:=mi_talker \\',
  '  --remap chatter:=/robot1/chatter',
].join('\n');

const challengeCode = [
  '# 1. Lanzar TurtleBot3 simulación (o robot real)',
  'export TURTLEBOT3_MODEL=burger',
  'ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py',
  '',
  '# 2. Inspeccionar el grafo completo',
  'ros2 node list',
  'ros2 topic list',
  'rqt_graph  # visualización gráfica',
  '',
  '# 3. Ver info de un nodo',
  'ros2 node info /turtlebot3_diff_drive',
  '',
  '# 4. Monitorear latencia de topics',
  'ros2 topic hz /scan           # frecuencia',
  'ros2 topic bw /camera/image_raw  # ancho de banda',
  '',
  '# 5. Ver QoS de publishers y subscribers',
  'ros2 topic info /scan --verbose',
  '',
  '# 6. Detectar QoS mismatches',
  'ros2 topic info /cmd_vel --verbose | grep -A5 "Reliability"',
].join('\n');

const discoveryPhase1Code = [
  '# Ver tráfico de descubrimiento en la red',
  'ros2 daemon stop && ros2 daemon start --verbose',
  '',
  '# En redes WiFi con muchos robots: usar Discovery Server',
  'export RMW_IMPLEMENTATION=rmw_fastrtps_cpp',
  'export FASTRTPS_DEFAULT_PROFILES_FILE=super_client.xml',
].join('\n');

const discoveryPhase2Code = [
  '# Forzar redescubrimiento manual',
  'ros2 daemon stop',
  'ros2 daemon start',
  '',
  '# Ver participantes DDS activos',
  'ros2 daemon status',
].join('\n');

const intraProcessCode = [
  '// Habilitar intra-process (zero-copy)',
  'auto options = rclcpp::NodeOptions()',
  '  .use_intra_process_comms(true);',
  'auto node = std::make_shared<CameraNode>(options);',
].join('\n');

const shmCode = [
  '<!-- fastdds_shm.xml -->',
  '<transport_descriptor>',
  '  <transport_id>SHM</transport_id>',
  '  <type>SHM</type>',
  '  <segment_size>104857600</segment_size>',
  '</transport_descriptor>',
].join('\n');

const executorSingleCode = [
  '// SingleThreaded — un callback a la vez',
  'rclcpp::executors::SingleThreadedExecutor exec;',
  'exec.add_node(node);',
  'exec.spin();',
].join('\n');

const executorMultiCode = [
  '// MultiThreaded — callbacks en paralelo',
  'rclcpp::executors::MultiThreadedExecutor exec(4); // 4 threads',
  'exec.add_node(node);',
  'exec.spin();',
  '',
  '// Callback Group: permito ejecución concurrente',
  'auto cbg = node->create_callback_group(',
  '  rclcpp::CallbackGroupType::Reentrant);',
].join('\n');

// ═══════════════════════════════════════════════════
// DATA ARRAYS
// ═══════════════════════════════════════════════════

const facts = [
  { icon: '🔗', label: 'DDS: estándar industrial usado en aviónica, finanzas y defensa — antes de robótica' },
  { icon: '⚡', label: 'Intra-process: latencia < 10 μs (1000× más rápido que UDP en red WiFi)' },
  { icon: '🤖', label: 'Domain ID: aísla hasta 102 robots en la misma red sin configuración extra' },
];

const graphCmds = [
  { cmd: 'ros2 node list',     color: '#4ade80', icon: 'list',         desc: 'Ver todos los nodos activos en el grafo' },
  { cmd: 'ros2 node info /nombre', color: '#fbbf24', icon: 'info',      desc: 'Publishers, subscribers y servicios de un nodo' },
  { cmd: 'ros2 topic list',    color: '#60a5fa', icon: 'topic',        desc: 'Todos los topics activos con sus tipos' },
  { cmd: 'rqt_graph',          color: '#c084fc', icon: 'share',        desc: 'Visualización gráfica interactiva del grafo' },
  { cmd: 'ros2 topic hz /scan',color: '#f97316', icon: 'speed',        desc: 'Frecuencia de publicación en tiempo real' },
  { cmd: 'ros2 topic bw /img', color: '#f87171', icon: 'network_check',desc: 'Ancho de banda consumido por el topic' },
];

const monoModules = ['Percepción', 'Control', 'Navegación', 'Planificación'];
const monoProblems = [
  'Fallo en un módulo detiene todo el sistema',
  'Acoplamiento fuerte — cambiar uno rompe otros',
  'Escalabilidad limitada: un solo proceso',
  'Debugging: stack traces entrelazados',
];
const distNodes = [
  { label: 'Percepción',   icon: 'videocam',         color: '#4ade80' },
  { label: 'Control',      icon: 'developer_board',  color: '#60a5fa' },
  { label: 'Navegación',   icon: 'explore',          color: '#fbbf24' },
  { label: 'Planificación',icon: 'psychology',        color: '#c084fc' },
];
const distBenefits = [
  'Fault isolation — fallo en un nodo no afecta a otros',
  'Escalabilidad horizontal: multi-core y multi-máquina',
  'Heterogeneidad: C++, Python y Rust en el mismo sistema',
  'Testing independiente por nodo (mocking de topics)',
];

const ddsLayers = [
  { title: 'Aplicación (ROS 2 API)', subtitle: 'Tu código de nodos', icon: 'code', color: '#c084fc', tags: ['rclcpp::Node', 'rclpy.Node'] },
  { title: 'RMW Interface', subtitle: 'Abstracción del vendor DDS', icon: 'layers', color: '#60a5fa', tags: ['rmw_fastrtps', 'rmw_cyclonedds'] },
  { title: 'DDS Implementation', subtitle: 'Motor de comunicación', icon: 'settings_ethernet', color: '#22d3ee', tags: ['FastDDS', 'CycloneDDS', 'RTI Connext'] },
  { title: 'Transporte', subtitle: 'Nivel de red físico', icon: 'wifi', color: '#4ade80', tags: ['UDP Multicast', 'TCP Unicast', 'Shared Memory'] },
];

const discoveryPhases = [
  {
    title: 'SPDP — Simple Participant Discovery',
    color: '#60a5fa',
    desc: 'Los participantes envían mensajes multicast UDP periódicos para anunciarse en la red. Automático, sin configuración.',
    warning: 'En WiFi: el tráfico escala O(N²) con muchos robots → usar Discovery Server para evitar "Discovery Storm"',
    code: discoveryPhase1Code,
  },
  {
    title: 'SEDP — Simple Endpoint Discovery',
    color: '#4ade80',
    desc: 'Una vez descubiertos los participantes, intercambian sus endpoints (publishers/subscribers) y negocian QoS.',
    warning: '',
    code: discoveryPhase2Code,
  },
];

const qosProfiles = [
  { policy: 'Reliability', icon: 'verified',        color: '#4ade80', options: ['RELIABLE', 'BEST_EFFORT'],      use: 'RELIABLE: comandos críticos. BEST_EFFORT: sensores alta frecuencia (pérdida aceptable)' },
  { policy: 'Durability',  icon: 'save',            color: '#60a5fa', options: ['VOLATILE', 'TRANSIENT_LOCAL'],  use: 'TRANSIENT_LOCAL: parámetros de config (late-joiners reciben el último valor)' },
  { policy: 'History',     icon: 'history',         color: '#fbbf24', options: ['KEEP_LAST(n)', 'KEEP_ALL'],     use: 'KEEP_LAST(1): solo el dato más reciente. KEEP_ALL: logging completo del sistema' },
  { policy: 'Deadline',    icon: 'timer',           color: '#c084fc', options: ['Duration'],                     use: 'Detecta publishers que no publican a la frecuencia esperada → alerta o fallo' },
  { policy: 'Lifespan',    icon: 'hourglass_empty', color: '#f97316', options: ['Duration'],                     use: 'Expira mensajes obsoletos (ej: obstacle map de hace 5s ya no es válido)' },
  { policy: 'Liveliness',  icon: 'favorite',        color: '#f87171', options: ['AUTOMATIC', 'MANUAL'],          use: 'Detecta nodos que han fallado silenciosamente sin desconectarse del grafo' },
];

const compatRules = [
  { pub: 'RELIABLE',    sub: 'RELIABLE',    ok: true,  label: 'Compatible ✓' },
  { pub: 'BEST_EFFORT', sub: 'BEST_EFFORT', ok: true,  label: 'Compatible ✓' },
  { pub: 'RELIABLE',    sub: 'BEST_EFFORT', ok: true,  label: 'Compatible (pub más estricto) ✓' },
  { pub: 'BEST_EFFORT', sub: 'RELIABLE',    ok: false, label: 'INCOMPATIBLE — sin comunicación ✗' },
];

const lifecycleStates = [
  { name: 'Unconfigured', sub: 'Inicial',  icon: 'radio_button_unchecked', color: '#94a3b8', active: false, transition: 'configure()' },
  { name: 'Inactive',     sub: 'Listo',    icon: 'pause_circle',           color: '#fbbf24', active: false, transition: 'activate()' },
  { name: 'Active',       sub: 'Corriendo',icon: 'play_circle',            color: '#4ade80', active: true,  transition: 'shutdown()' },
  { name: 'Finalized',    sub: 'Terminado', icon: 'stop_circle',           color: '#60a5fa', active: false, transition: '' },
];

const executors = [
  {
    name: 'SingleThreadedExecutor',
    badge: 'Default',
    icon: 'arrow_right_alt',
    color: '#60a5fa',
    desc: 'Un hilo procesa todos los callbacks en orden. Seguro para acceso a variables compartidas, pero limita el throughput en sistemas con muchos sensores.',
    callbacks: [
      { label: 'timer_cb', color: '#60a5fa', active: true },
      { label: 'scan_cb',  color: '#4ade80', active: false },
      { label: 'odom_cb',  color: '#fbbf24', active: false },
    ],
    code: executorSingleCode,
  },
  {
    name: 'MultiThreadedExecutor',
    badge: 'Alto rendimiento',
    icon: 'account_tree',
    color: '#4ade80',
    desc: 'Pool de hilos para callbacks en paralelo. Requiere sincronización explícita. Ideal para sistemas con procesamiento intensivo de imagen o LIDAR.',
    callbacks: [
      { label: 'timer_cb', color: '#4ade80', active: true },
      { label: 'scan_cb',  color: '#4ade80', active: true },
      { label: 'img_cb',   color: '#4ade80', active: true },
    ],
    code: executorMultiCode,
  },
];

const cbGroups = [
  { name: 'MutuallyExclusive (default)', color: '#f87171', desc: 'Solo un callback del grupo se ejecuta a la vez. Protege automáticamente variables compartidas.' },
  { name: 'Reentrant', color: '#4ade80', desc: 'Múltiples callbacks del grupo pueden ejecutarse simultáneamente. Máximo throughput, requiere thread-safety manual.' },
];

const domains = [
  { id: 0, name: 'Robot A', color: '#60a5fa', nodes: ['/camera_node', '/lidar_node', '/nav2_node'] },
  { id: 1, name: 'Robot B', color: '#4ade80', nodes: ['/camera_node', '/lidar_node', '/nav2_node'] },
];

const namespaceTree = [
  { path: '/robot1',                   type: 'namespace',  color: '#60a5fa', depth: 0 },
  { path: '/robot1/camera_node',        type: 'node',       color: '#4ade80', depth: 1 },
  { path: '/robot1/image_raw',          type: 'topic',      color: '#fbbf24', depth: 2 },
  { path: '/robot1/camera_info',        type: 'topic',      color: '#fbbf24', depth: 2 },
  { path: '/robot1/nav2_node',          type: 'node',       color: '#4ade80', depth: 1 },
  { path: '/robot1/cmd_vel',            type: 'topic',      color: '#fbbf24', depth: 2 },
  { path: '/robot2',                   type: 'namespace',  color: '#c084fc', depth: 0 },
  { path: '/robot2/camera_node',        type: 'node',       color: '#4ade80', depth: 1 },
];

const perfBars = [
  { mode: 'Intra-Process (mismo proceso)',   color: '#4ade80', pct: 2,   latency: '< 10 μs',   delay: '0.1s' },
  { mode: 'Shared Memory (misma máquina)',   color: '#fbbf24', pct: 15,  latency: '50-200 μs',  delay: '0.2s' },
  { mode: 'UDP Localhost (mismo OS)',        color: '#f97316', pct: 40,  latency: '200-800 μs', delay: '0.3s' },
  { mode: 'UDP Red Local (Ethernet)',        color: '#f87171', pct: 70,  latency: '0.5-5 ms',   delay: '0.4s' },
  { mode: 'UDP WiFi (wireless)',             color: '#ef4444', pct: 100, latency: '2-20 ms',    delay: '0.5s' },
];

const perfCards = [
  {
    title: 'Intra-Process',
    icon: 'memory',
    color: '#4ade80',
    metrics: [
      { label: 'Latencia', value: '< 10 μs',  good: true  },
      { label: 'Throughput',value: '> 10 GB/s',good: true  },
      { label: 'CPU',       value: 'Mínimo',   good: true  },
    ],
    desc: 'Comunicación por punteros compartidos dentro del mismo proceso. Zero-copy cuando es posible. Requiere NodeOptions::use_intra_process_comms(true).',
    lang: 'cpp',
    code: intraProcessCode,
  },
  {
    title: 'Shared Memory',
    icon: 'storage',
    color: '#60a5fa',
    metrics: [
      { label: 'Latencia', value: '50-200 μs', good: false, bad: false },
      { label: 'Throughput',value: '1-5 GB/s',  good: true  },
      { label: 'CPU',       value: 'Bajo',       good: true  },
    ],
    desc: 'Memoria compartida entre procesos en la misma máquina (FastDDS SHM transport). Evita copias de red pero requiere configuración del descriptor de transporte.',
    lang: 'xml',
    code: shmCode,
  },
];

const benchRows = [
  { size: '1 KB',   intra: '8 μs',   shm: '45 μs',   udp: '120 μs' },
  { size: '100 KB', intra: '12 μs',  shm: '180 μs',  udp: '850 μs' },
  { size: '1 MB',   intra: '45 μs',  shm: '1.2 ms',  udp: '8.5 ms' },
  { size: '10 MB',  intra: '380 μs', shm: '9.8 ms',  udp: '85 ms'  },
];

const commonErrors = reactive([
  {
    type: 'QoS Mismatch silencioso — sin datos y sin error',
    summary: 'El subscriber existe, el publisher existe, pero nunca llegan mensajes',
    color: '#f87171',
    cause: 'BEST_EFFORT publisher + RELIABLE subscriber = incompatibles. ROS 2 no lanza error, simplemente no conecta.',
    code: [
      '# Diagnosticar QoS mismatch',
      'ros2 topic info /mi_topic --verbose',
      '# Buscar: "Incompatible QoS" en la salida',
      '',
      '# Ver eventos de QoS en tiempo real',
      'ros2 topic echo /rosout | grep -i "qos"',
    ].join('\n'),
    fix: 'Asegúrate de que publisher y subscriber usen el mismo nivel de Reliability. RELIABLE pub puede hablar con BEST_EFFORT sub, pero no al revés.',
    open: false,
  },
  {
    type: 'Discovery Storm en red WiFi con múltiples robots',
    summary: 'La red se satura cuando hay más de 4-5 robots en el mismo segmento WiFi',
    color: '#fbbf24',
    cause: 'SPDP usa multicast UDP. Con N robots, el tráfico de descubrimiento escala O(N²). En WiFi, el multicast es especialmente costoso.',
    code: [
      '# Solución: usar Discovery Server (FastDDS)',
      'export RMW_IMPLEMENTATION=rmw_fastrtps_cpp',
      '',
      '# Servidor de descubrimiento (en un nodo central)',
      'fastdds discovery --server-id 0',
      '',
      '# Cada robot apunta al servidor',
      'export ROS_DISCOVERY_SERVER="192.168.1.100:11811"',
    ].join('\n'),
    fix: 'Usa Discovery Server de FastDDS para centralizar el descubrimiento. Elimina el multicast y reduce el tráfico a O(N) por robot.',
    open: false,
  },
  {
    type: 'Lifecycle node bloqueado en Unconfigured',
    summary: 'ros2 lifecycle set /mi_nodo configure falla o no responde',
    color: '#c084fc',
    cause: 'on_configure() lanza una excepción no capturada, o tarda demasiado inicializando hardware. El nodo queda bloqueado.',
    code: [
      '# Ver el estado actual del lifecycle node',
      'ros2 lifecycle get /mi_nodo',
      '',
      '# Forzar transición a cleanup si está bloqueado',
      'ros2 lifecycle set /mi_nodo cleanup',
      '',
      '# Ver logs del nodo para encontrar el error',
      'ros2 topic echo /rosout | grep mi_nodo',
    ].join('\n'),
    fix: 'Envuelve la inicialización de hardware en try-catch en on_configure(). Retorna CallbackReturn::FAILURE si falla — el sistema puede reintentar o limpiar.',
    open: false,
  },
  {
    type: 'MultiThreadedExecutor con variables compartidas corrompidas',
    summary: 'Datos inconsistentes o crashes aleatorios en nodos con muchos callbacks',
    color: '#f97316',
    cause: 'Dos callbacks del grupo Reentrant acceden a la misma variable simultáneamente sin mutex. Race condition.',
    code: [
      '// Solución: usar mutex o MutuallyExclusive callback group',
      '#include <mutex>',
      'std::mutex data_mutex_;',
      '',
      'void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {',
      '  std::lock_guard<std::mutex> lock(data_mutex_);',
      '  latest_scan_ = msg;  // protegido',
      '}',
    ].join('\n'),
    fix: 'Usa MutuallyExclusive callback groups para callbacks que comparten estado. Solo usa Reentrant cuando los callbacks son stateless.',
    open: false,
  },
  {
    type: 'Nodo con Domain ID 0 no ve a otro robot',
    summary: 'ros2 node list no muestra los nodos del robot B aunque están en la misma red',
    color: '#60a5fa',
    cause: 'El robot B tiene ROS_DOMAIN_ID diferente (o no configurado). El default es 0 pero puede haber sido cambiado.',
    code: [
      '# Verificar Domain ID en cada terminal/robot',
      'echo $ROS_DOMAIN_ID',
      '# Si no aparece nada = Domain 0 (default)',
      '',
      '# En ambos robots deben coincidir',
      'export ROS_DOMAIN_ID=42   # robot A',
      'export ROS_DOMAIN_ID=42   # robot B (misma red)',
      '',
      '# Agregarlo al .bashrc para que persista',
      'echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc',
    ].join('\n'),
    fix: 'Asegúrate de que todos los robots que deben comunicarse tengan exactamente el mismo ROS_DOMAIN_ID. Añádelo al .bashrc para que persista.',
    open: false,
  },
]);

const challengeSteps = [
  { num: 1, color: '#4ade80', text: 'Lanza el simulador TurtleBot3 y lista todos los nodos y topics activos' },
  { num: 2, color: '#60a5fa', text: 'Usa rqt_graph para visualizar el grafo completo y guarda una captura' },
  { num: 3, color: '#fbbf24', text: 'Mide la frecuencia (hz) y ancho de banda (bw) de al menos 3 topics' },
  { num: 4, color: '#c084fc', text: 'Crea un subscriber con QoS RELIABLE al topic /scan (que publica BEST_EFFORT) y observa el mismatch' },
  { num: 5, color: '#f87171', text: 'Configura un segundo terminal con Domain ID=1 y confirma que no ve los nodos del simulador' },
];

const challengeHints = [
  'ros2 topic list -t muestra el tipo de mensaje junto al topic',
  'Para el QoS mismatch: ros2 topic info /scan --verbose muestra Offered QoS vs Requested QoS',
  'rqt_graph tiene un checkbox "Namespace clusters" para ver namespaces agrupados',
  'ros2 node info /nombre muestra también los Callback Groups del nodo',
  'Para ver lifecycle: ros2 lifecycle nodes lista todos los nodos lifecycle activos',
];

const summaryItems = [
  { term: 'Computation Graph',      desc: 'Red de nodos que publican/suscriben topics, servicios y acciones',      note: 'rqt_graph lo visualiza',           color: '#4ade80' },
  { term: 'DDS Middleware',         desc: 'Capa de comunicación publish-subscribe sobre UDP/TCP con QoS',           note: 'FastDDS por defecto en Jazzy',       color: '#60a5fa' },
  { term: 'QoS Profiles',           desc: '6+ políticas configurables: Reliability, Durability, History...',        note: 'RELIABLE pub ≥ BEST_EFFORT sub',    color: '#c084fc' },
  { term: 'Lifecycle Nodes',        desc: 'Nodos con estados (Unconfigured→Inactive→Active→Finalized)',              note: 'Startup/shutdown determinista',     color: '#f87171' },
  { term: 'Executors',              desc: 'SingleThreaded (seguro) o MultiThreaded (rendimiento)',                   note: 'Callback Groups controlan concurrencia', color: '#fbbf24' },
  { term: 'Domain ID',              desc: 'Aísla grupos de robots (0-101). Default: 0',                             note: 'export ROS_DOMAIN_ID=42',          color: '#f97316' },
  { term: 'Namespace',              desc: 'Prefijo jerárquico para nodos y topics: /robot1/cmd_vel',                 note: '--ros-args --remap __ns:=/robot1', color: '#4ade80' },
  { term: 'Intra-Process',          desc: 'Comunicación < 10μs por punteros compartidos dentro del proceso',        note: 'use_intra_process_comms(true)',    color: '#60a5fa' },
  { term: 'Shared Memory',          desc: 'Comunicación 50-200μs entre procesos en la misma máquina',              note: 'FastDDS SHM transport',            color: '#c084fc' },
];
</script>

<style scoped>
/* ══════════════════════════════════════════
   BASE
══════════════════════════════════════════ */
.section-group { margin-bottom: 3.5rem; }

code {
  background: var(--bg-code); color: var(--text-code);
  padding: 2px 7px; border-radius: 5px;
  font-family: 'Fira Code', monospace; font-size: .9em;
}
.cmd-badge {
  display: inline-flex; align-items: center; justify-content: center;
  width: 28px; height: 28px; border-radius: 8px;
  font-size: .75rem; font-weight: 800; margin-right: 8px; vertical-align: middle;
}
.cmd-badge.green  { background: rgba( 74,222,128,.15); color: #4ade80; }
.cmd-badge.amber  { background: rgba(251,191, 36,.15); color: #fbbf24; }
.cmd-badge.cyan   { background: rgba( 34,211,238,.15); color: #22d3ee; }
.cmd-badge.purple { background: rgba(192,132,252,.15); color: #c084fc; }
.cmd-badge.red    { background: rgba(248,113,113,.15); color: #f87171; }

/* FACT PILLS */
.fact-pills { display: flex; gap: 10px; flex-wrap: wrap; }
.fact-pill {
  display: flex; align-items: center; gap: 8px;
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-radius: 999px; padding: 7px 16px; font-size: .84rem; color: var(--text-secondary);
  transition: transform .2s;
}
.fact-pill:hover { transform: translateY(-2px); }
.fp-icon { font-size: 1rem; }

/* ══════════════════════════════════════════
   COMPUTATION GRAPH
══════════════════════════════════════════ */
.cg-wrapper {
  background: var(--bg-deep, #0d1117); border: 1px solid var(--border-subtle);
  border-radius: 16px; padding: 1.5rem; overflow: hidden;
}
.cg-title { font-size: .82rem; font-weight: 600; color: var(--text-muted); text-align: center; margin-bottom: 12px; }
.cg-svg { width: 100%; height: auto; display: block; }

/* Animated flow lines */
@keyframes cgFlow { to { stroke-dashoffset: -100; } }
.cg-flow { animation: cgFlow 2s linear infinite; }
.cg-f1 { animation-duration: 1.8s; }
.cg-f2 { animation-duration: 2.2s; animation-delay: .5s; }
.cg-f3 { animation-duration: 1.6s; animation-delay: .3s; }
.cg-f4 { animation-duration: 2.4s; animation-delay: .8s; }
.cg-f5 { animation-duration: 1.4s; animation-delay: .2s; }

.cg-legend { display: flex; flex-wrap: wrap; gap: 14px; justify-content: center; margin-top: 12px; }
.cgl-item { display: flex; align-items: center; gap: 6px; font-size: .78rem; color: var(--text-muted); }
.cgl-dot  { width: 10px; height: 10px; border-radius: 50%; background: var(--cgl-c); flex-shrink: 0; }

/* CMD SHOWCASE */
.cmd-showcase { display: grid; grid-template-columns: repeat(3, 1fr); gap: 10px; }
.cs-item {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-left: 3px solid var(--cs-color); border-radius: 10px;
  padding: 10px 12px; display: flex; flex-direction: column; gap: 5px; min-width: 0;
}
.csi-header { display: flex; align-items: center; gap: 7px; flex-wrap: wrap; }
.csi-cmd { font-size: .78rem; font-weight: 700; color: var(--cs-color); background: none; padding: 0; word-break: break-all; }
.csi-desc { font-size: .76rem; color: var(--text-muted); line-height: 1.4; }

/* ══════════════════════════════════════════
   ARCH COMPARISON
══════════════════════════════════════════ */
.arch-grid { display: grid; grid-template-columns: repeat(2, 1fr); gap: 16px; }
.arch-card {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-top: 3px solid var(--ac-color); border-radius: 16px;
  padding: 1.5rem; display: flex; flex-direction: column; gap: 14px; min-width: 0;
}
.acc-header { display: flex; align-items: center; gap: 10px; }
.acc-icon { width: 42px; height: 42px; border-radius: 12px; display: flex; align-items: center; justify-content: center; flex-shrink: 0; }
.acc-title { font-size: .95rem; font-weight: 700; color: var(--text-primary); }

.mono-block {
  background: var(--bg-surface-hover); border-radius: 10px; padding: 12px;
  display: flex; flex-direction: column; gap: 5px; align-items: center;
}
.mono-module {
  width: 100%; text-align: center; padding: 6px 10px;
  background: var(--bg-surface); border-radius: 6px;
  font-size: .84rem; color: var(--text-secondary); font-weight: 600;
}
.mono-coupling {
  font-size: .74rem; color: var(--text-muted); margin-top: 4px; text-align: center;
  font-style: italic;
}

.dist-graph {
  display: grid; grid-template-columns: repeat(2, 1fr); gap: 8px;
  background: var(--bg-surface-hover); border-radius: 10px; padding: 12px;
}
.dist-node {
  display: flex; align-items: center; gap: 7px;
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-left: 3px solid var(--dn-color); border-radius: 8px;
  padding: 8px 10px; font-size: .82rem; font-weight: 600; color: var(--text-primary);
  animation: nodeFloat 3s ease-in-out infinite;
}
.dist-node:nth-child(2) { animation-delay: .75s; }
.dist-node:nth-child(3) { animation-delay: 1.5s; }
.dist-node:nth-child(4) { animation-delay: 2.25s; }
@keyframes nodeFloat {
  0%,100% { transform: translateY(0); }
  50% { transform: translateY(-3px); }
}

.acc-point { display: flex; align-items: flex-start; gap: 8px; font-size: .83rem; color: var(--text-secondary); padding: 5px 0; }

.insight-box {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-left: 3px solid var(--ib-color); border-radius: 12px; padding: 1.1rem 1.25rem;
}
.ib-header { display: flex; align-items: center; gap: 8px; margin-bottom: 8px; font-size: .9rem; color: var(--text-primary); }
.ib-desc { font-size: .85rem; color: var(--text-secondary); line-height: 1.6; margin: 0; }

/* ══════════════════════════════════════════
   DDS STACK
══════════════════════════════════════════ */
.dds-stack { display: flex; flex-direction: column; gap: 0; background: var(--bg-surface); border: 1px solid var(--border-subtle); border-radius: 16px; overflow: hidden; }
.dds-layer {
  display: flex; align-items: center; justify-content: space-between; gap: 12px;
  padding: 1.1rem 1.5rem; border-bottom: 1px solid var(--border-subtle);
  border-left: 3px solid var(--dl-color);
  animation: layerSlide .5s ease backwards;
  animation-delay: var(--dl-delay);
  position: relative;
}
.dds-layer:last-child { border-bottom: none; }
@keyframes layerSlide {
  from { opacity: 0; transform: translateX(-16px); }
  to   { opacity: 1; transform: translateX(0); }
}
.dll-left { display: flex; align-items: center; gap: 12px; min-width: 0; }
.dll-icon { width: 40px; height: 40px; border-radius: 10px; display: flex; align-items: center; justify-content: center; background: color-mix(in srgb, var(--dl-color) 12%, transparent); flex-shrink: 0; }
.dll-title    { font-size: .9rem; font-weight: 700; }
.dll-subtitle { font-size: .78rem; color: var(--text-muted); }
.dll-right    { display: flex; gap: 6px; flex-wrap: wrap; justify-content: flex-end; flex-shrink: 0; }
.dll-tag      { font-family: 'Fira Code', monospace; font-size: .72rem; padding: 2px 7px; border-radius: 5px; border: 1px solid; background: none; white-space: nowrap; }
.dll-arrow {
  position: absolute; bottom: -16px; left: 50%; transform: translateX(-50%);
  z-index: 2; display: flex; flex-direction: column; align-items: center; gap: 2px;
}
.dlla-packet {
  width: 10px; height: 10px; border-radius: 50%;
  background: var(--pack-c); opacity: 0;
  animation: packetDrop 2s ease infinite;
}
@keyframes packetDrop {
  0%   { opacity: 0; transform: translateY(0); }
  20%  { opacity: 1; }
  80%  { opacity: 1; }
  100% { opacity: 0; transform: translateY(20px); }
}

/* DISCOVERY GRID */
.discovery-grid { display: grid; grid-template-columns: repeat(2, 1fr); gap: 14px; }
.disc-card {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-top: 3px solid var(--dp-color); border-radius: 14px;
  display: flex; gap: 14px; padding: 1.25rem; min-width: 0;
}
.dpc-number {
  min-width: 38px; height: 38px; border-radius: 50%;
  display: flex; align-items: center; justify-content: center;
  font-size: 1.1rem; font-weight: 800; flex-shrink: 0;
}
.dpc-body { flex: 1; min-width: 0; display: flex; flex-direction: column; gap: 8px; }
.dpc-title { font-size: .9rem; font-weight: 700; }
.dpc-desc  { font-size: .83rem; color: var(--text-secondary); line-height: 1.5; margin: 0; }
.dpc-warn  { display: flex; align-items: flex-start; gap: 6px; font-size: .78rem; color: var(--text-secondary); background: color-mix(in srgb, #fbbf24 8%, transparent); border-radius: 6px; padding: 6px 8px; }

/* ══════════════════════════════════════════
   QoS
══════════════════════════════════════════ */
.qos-cards { display: grid; grid-template-columns: repeat(3, 1fr); gap: 10px; }
.qos-card {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-top: 3px solid var(--qos-color); border-radius: 12px;
  padding: 12px 14px; display: flex; flex-direction: column; gap: 8px; min-width: 0;
}
.qosc-header { display: flex; align-items: center; gap: 8px; }
.qosc-policy { font-size: .88rem; font-weight: 800; color: var(--qos-color); background: none; padding: 0; }
.qosc-options { display: flex; flex-wrap: wrap; gap: 5px; }
.qosc-opt {
  font-family: 'Fira Code', monospace; font-size: .72rem;
  background: var(--bg-surface-hover); color: var(--text-code);
  padding: 2px 7px; border-radius: 5px;
}
.qosc-use { font-size: .78rem; color: var(--text-muted); line-height: 1.45; }

.compat-box { background: var(--bg-surface); border: 1px solid var(--border-subtle); border-radius: 14px; padding: 1.25rem; }
.cb-title { display: flex; align-items: center; gap: 8px; font-size: .9rem; font-weight: 700; color: var(--text-primary); margin-bottom: 14px; }
.compat-grid { display: flex; flex-direction: column; gap: 8px; }
.compat-item { display: flex; align-items: center; gap: 10px; font-size: .84rem; flex-wrap: wrap; padding: 7px 10px; background: var(--bg-surface-hover); border-radius: 8px; }
.ci-indicator { flex-shrink: 0; }
.ci-pub, .ci-sub { font-size: .82rem; font-weight: 700; }
.ci-label { color: var(--text-muted); font-size: .78rem; margin-left: auto; }

/* ══════════════════════════════════════════
   LIFECYCLE
══════════════════════════════════════════ */
.lc-machine { background: var(--bg-surface); border: 1px solid var(--border-subtle); border-radius: 16px; padding: 1.5rem; }
.lcm-track { display: flex; align-items: center; flex-wrap: wrap; gap: 0; }
.lcm-item { display: flex; align-items: center; gap: 0; }
.lcm-state {
  display: flex; flex-direction: column; align-items: center; gap: 6px;
  background: var(--bg-surface-hover); border: 2px solid color-mix(in srgb, var(--lcs-color) 30%, transparent);
  border-radius: 14px; padding: 14px 18px; min-width: 100px;
  transition: transform .2s;
}
.lcm-state:hover { transform: translateY(-3px); }
.lcm-active {
  border-color: var(--lcs-color);
  background: color-mix(in srgb, var(--lcs-color) 10%, var(--bg-surface));
  animation: lcPulse 2s ease-in-out infinite;
}
@keyframes lcPulse {
  0%,100% { box-shadow: 0 0 0 0 transparent; }
  50% { box-shadow: 0 0 0 6px transparent; }
}
.lcms-name { font-size: .82rem; font-weight: 800; color: var(--text-primary); }
.lcms-sub  { font-size: .72rem; color: var(--text-muted); }
.lcm-transition { display: flex; flex-direction: column; align-items: center; gap: 2px; padding: 0 8px; }
.lcmt-fn { font-family: 'Fira Code', monospace; font-size: .68rem; white-space: nowrap; font-weight: 700; }
.lcm-back-arrows { display: flex; flex-direction: column; gap: 6px; padding-top: 12px; border-top: 1px solid var(--border-subtle); }
.lcm-back { font-size: .8rem; color: var(--text-muted); display: flex; align-items: center; gap: 8px; }
.lcm-back::before { content: '↩'; color: var(--lb-color); font-weight: 700; }

/* ══════════════════════════════════════════
   EXECUTORS
══════════════════════════════════════════ */
.exec-grid { display: grid; grid-template-columns: repeat(2, 1fr); gap: 16px; }
.exec-card {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-top: 3px solid var(--ex-color); border-radius: 16px;
  padding: 1.5rem; display: flex; flex-direction: column; gap: 14px; min-width: 0;
}
.exc-header { display: flex; align-items: flex-start; gap: 12px; }
.exc-icon { width: 48px; height: 48px; border-radius: 14px; display: flex; align-items: center; justify-content: center; flex-shrink: 0; }
.exc-name { font-family: 'Fira Code', monospace; font-size: .88rem; font-weight: 800; color: var(--ex-color); display: block; background: none; padding: 0; }
.exc-badge { font-size: .72rem; font-weight: 700; padding: 2px 8px; border-radius: 6px; display: inline-block; margin-top: 4px; }
.exc-desc { font-size: .84rem; color: var(--text-secondary); line-height: 1.55; margin: 0; }
.exc-queue { display: flex; flex-direction: column; gap: 6px; }
.excq-label { font-size: .78rem; font-weight: 600; color: var(--text-muted); }
.excq-track { display: flex; gap: 6px; flex-wrap: wrap; }
.excq-item {
  font-family: 'Fira Code', monospace; font-size: .76rem; padding: 4px 10px;
  background: color-mix(in srgb, var(--eqi-c) 12%, transparent);
  border: 1px solid color-mix(in srgb, var(--eqi-c) 30%, transparent);
  color: var(--eqi-c); border-radius: 6px;
}
.excq-active {
  animation: cbPulse 1.2s ease infinite;
  animation-delay: inherit;
}
@keyframes cbPulse {
  0%,100% { opacity: 1; transform: scale(1); }
  50% { opacity: .7; transform: scale(.95); }
}

.cbgroup-box { background: var(--bg-surface); border: 1px solid var(--border-subtle); border-radius: 14px; padding: 1.25rem; }
.cbg-title { display: flex; align-items: center; gap: 8px; font-size: .9rem; font-weight: 700; color: var(--text-primary); margin-bottom: 12px; }
.cbg-grid { display: grid; grid-template-columns: repeat(2, 1fr); gap: 10px; }
.cbg-card { background: var(--bg-surface-hover); border-left: 3px solid var(--cbg-color); border-radius: 10px; padding: 10px 14px; }
.cbgc-name { font-family: 'Fira Code', monospace; font-size: .8rem; font-weight: 700; margin-bottom: 5px; }
.cbgc-desc { font-size: .8rem; color: var(--text-muted); line-height: 1.4; }

/* ══════════════════════════════════════════
   DOMAIN + NAMESPACE
══════════════════════════════════════════ */
.domain-visual {
  display: grid; grid-template-columns: 1fr auto 1fr; gap: 20px; align-items: center;
  background: var(--bg-surface); border: 1px solid var(--border-subtle); border-radius: 16px; padding: 1.5rem;
}
.domain-group { display: flex; flex-direction: column; gap: 10px; }
.domg-header {
  display: flex; align-items: center; gap: 8px; font-size: .88rem; font-weight: 700;
  color: var(--dom-color); background: color-mix(in srgb, var(--dom-color) 10%, transparent);
  border-radius: 8px; padding: 8px 12px;
}
.domg-nodes { display: flex; flex-direction: column; gap: 6px; }
.domg-node {
  font-family: 'Fira Code', monospace; font-size: .82rem;
  background: var(--bg-surface-hover); border: 1px solid var(--border-subtle);
  border-radius: 6px; padding: 7px 12px; color: var(--text-secondary); text-align: center;
}
.domain-block { display: flex; flex-direction: column; align-items: center; gap: 4px; }
.dombl-text { font-size: .78rem; font-weight: 700; color: var(--text-muted); }

.ns-visual { background: var(--bg-surface); border: 1px solid var(--border-subtle); border-radius: 14px; padding: 1.25rem; }
.nsv-title { font-size: .88rem; font-weight: 700; color: var(--text-secondary); margin-bottom: 12px; }
.nsv-tree { display: flex; flex-direction: column; gap: 3px; }
.nst-item { display: flex; align-items: center; gap: 8px; padding: 4px 0; }
.nsti-line { width: 12px; height: 1px; background: var(--border-medium); flex-shrink: 0; }
.nsti-path { font-size: .82rem; font-weight: 600; background: none; padding: 0; }
.nsti-type { font-size: .72rem; color: var(--text-muted); background: var(--bg-surface-hover); padding: 1px 6px; border-radius: 4px; }

/* ══════════════════════════════════════════
   PERFORMANCE
══════════════════════════════════════════ */
.perf-bars { display: flex; flex-direction: column; gap: 12px; background: var(--bg-surface); border: 1px solid var(--border-subtle); border-radius: 14px; padding: 1.5rem; }
.pb-row { display: grid; grid-template-columns: 220px 1fr 90px; gap: 12px; align-items: center; }
.pbr-label { font-size: .82rem; color: var(--text-secondary); }
.pbr-track { height: 10px; background: var(--bg-surface-hover); border-radius: 5px; overflow: hidden; }
.pbr-fill  {
  height: 100%; background: var(--pb-color); border-radius: 5px;
  transform-origin: left;
  animation: barGrow .8s ease backwards;
  animation-delay: var(--pb-delay, 0s);
  width: var(--pb-width);
}
@keyframes barGrow {
  from { transform: scaleX(0); }
  to   { transform: scaleX(1); }
}
.pbr-value { font-family: 'Fira Code', monospace; font-size: .8rem; font-weight: 700; text-align: right; }

.perf-cards { display: grid; grid-template-columns: repeat(2, 1fr); gap: 14px; }
.perf-card {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-top: 3px solid var(--pc-color); border-radius: 14px;
  padding: 1.25rem; display: flex; flex-direction: column; gap: 10px; min-width: 0;
}
.pcc-header { display: flex; align-items: center; gap: 10px; }
.pcc-title  { font-size: .92rem; font-weight: 700; color: var(--text-primary); }
.pcc-metrics { display: flex; flex-direction: column; gap: 6px; }
.pccm-row { display: flex; justify-content: space-between; padding: 6px 10px; background: var(--bg-surface-hover); border-radius: 6px; }
.pccm-label { font-size: .82rem; color: var(--text-muted); }
.pccm-val   { font-family: 'Fira Code', monospace; font-size: .82rem; font-weight: 700; }
.pcc-desc   { font-size: .82rem; color: var(--text-secondary); line-height: 1.5; margin: 0; }

.bench-table { background: var(--bg-surface); border: 1px solid var(--border-subtle); border-radius: 12px; overflow: hidden; }
.bt-header, .bt-row { display: grid; grid-template-columns: 1fr 1fr 1fr 1fr; }
.bt-header { background: var(--bg-surface-hover); border-bottom: 1px solid var(--border-medium); }
.bt-row    { border-bottom: 1px solid var(--border-subtle); }
.bt-row:last-child { border-bottom: none; }
.bt-cell { padding: 10px 14px; font-family: 'Fira Code', monospace; font-size: .82rem; color: var(--text-secondary); }
.bt-cell-hd { color: var(--text-primary); font-weight: 600; }
.bt-header .bt-cell { font-size: .8rem; font-weight: 700; }
.bench-note { font-size: .78rem; color: var(--text-muted); font-style: italic; margin-top: 6px; padding-left: 4px; }

/* ══════════════════════════════════════════
   ERROR ACCORDION
══════════════════════════════════════════ */
.error-list { display: flex; flex-direction: column; gap: 10px; }
.error-item { background: var(--bg-surface); border: 1px solid var(--border-subtle); border-left: 3px solid var(--err-color); border-radius: 12px; overflow: hidden; }
.err-header { display: flex; align-items: center; justify-content: space-between; padding: .9rem 1.25rem; cursor: pointer; gap: 12px; transition: background .2s; }
.err-header:hover { background: var(--bg-surface-hover); }
.err-left   { display: flex; align-items: flex-start; gap: 10px; min-width: 0; }
.err-num    { min-width: 26px; width: 26px; height: 26px; border-radius: 50%; flex-shrink: 0; font-size: .8rem; font-weight: 800; display: flex; align-items: center; justify-content: center; }
.err-type    { font-size: .82rem; font-weight: 700; color: var(--text-primary); margin-bottom: 2px; }
.err-summary { font-size: .78rem; color: var(--text-muted); }
.err-body  { padding: .9rem 1.4rem 1.1rem; border-top: 1px solid var(--border-subtle); display: flex; flex-direction: column; gap: 10px; }
.err-cause { font-size: .86rem; color: var(--text-secondary); display: flex; align-items: flex-start; gap: 6px; }
.err-fix   { font-size: .85rem; color: var(--text-secondary); display: flex; align-items: flex-start; gap: 6px; }

/* ══════════════════════════════════════════
   CHALLENGE
══════════════════════════════════════════ */
.challenge-box { background: var(--bg-surface); border: 1px solid var(--border-subtle); border-radius: 20px; padding: 1.75rem; border-top: 3px solid #f59e0b; }
.challenge-header { display: flex; align-items: flex-start; gap: 1rem; flex-wrap: wrap; }
.challenge-icon { width: 52px; height: 52px; background: rgba(245,158,11,.15); border-radius: 14px; display: flex; align-items: center; justify-content: center; flex-shrink: 0; }
.challenge-title { font-size: 1.05rem; font-weight: 700; color: var(--text-primary); margin-bottom: 4px; }
.challenge-subtitle { font-size: .9rem; color: var(--text-secondary); }
.challenge-badge { margin-left: auto; font-size: .72rem; font-weight: 800; padding: 4px 12px; border-radius: 999px; white-space: nowrap; background: rgba(96,165,250,.12); color: #60a5fa; border: 1px solid rgba(96,165,250,.3); }
.challenge-steps { background: var(--bg-surface-hover); border-radius: 12px; padding: 1rem 1.25rem; }
.cs-title { font-size: .88rem; font-weight: 700; color: var(--text-secondary); margin-bottom: 10px; }
.cs-list  { display: flex; flex-direction: column; gap: 8px; }
.cs-item  { display: flex; align-items: flex-start; gap: 10px; }
.cs-num   { min-width: 26px; width: 26px; height: 26px; border-radius: 50%; flex-shrink: 0; font-size: .82rem; font-weight: 800; color: #0d1117; display: flex; align-items: center; justify-content: center; }
.cs-text  { font-size: .87rem; color: var(--text-secondary); padding-top: 3px; }
:deep(.answer-header) { background: rgba(34,197,94,.08); border: 1px solid rgba(34,197,94,.25); border-radius: 10px; color: #22c55e; }
.answer-body { background: var(--bg-surface-hover); padding: 1.1rem 1.25rem; border-radius: 0 0 10px 10px; display: flex; flex-direction: column; gap: 8px; }
.answer-row  { display: flex; align-items: baseline; gap: 8px; font-size: .88rem; color: var(--text-secondary); }

/* ══════════════════════════════════════════
   VIDEO
══════════════════════════════════════════ */
.video-card    { background: var(--bg-surface); border: 1px solid var(--border-subtle); border-radius: 16px; padding: 1.25rem; overflow: hidden; }
.video-wrapper { position: relative; padding-bottom: 56.25%; height: 0; overflow: hidden; border-radius: 10px; background: #000; }
.video-wrapper iframe { position: absolute; top: 0; left: 0; width: 100%; height: 100%; }
.video-caption { display: flex; align-items: center; margin-top: 12px; font-size: .82rem; color: var(--text-muted); padding: 8px 12px; background: var(--bg-surface-hover); border-radius: 8px; }

/* ══════════════════════════════════════════
   SUMMARY
══════════════════════════════════════════ */
.summary-grid { display: grid; grid-template-columns: repeat(3, 1fr); gap: 12px; }
.summary-card {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-left: 4px solid var(--sc-color); border-radius: 12px;
  padding: 1rem 1.25rem; display: flex; flex-direction: column; gap: 6px;
  transition: transform .2s;
}
.summary-card:hover { transform: translateY(-3px); }
.sc-term { display: block; font-size: .84rem; font-weight: 700; color: var(--sc-color); background: none; padding: 0; }
.sc-desc { font-size: .8rem; color: var(--text-secondary); line-height: 1.4; }
.sc-note { font-family: 'Fira Code', monospace; font-size: .72rem; color: var(--text-muted); }

/* ══════════════════════════════════════════
   CTA
══════════════════════════════════════════ */
.final-cta { background: var(--bg-surface); border: 1px solid var(--border-subtle); border-radius: 24px; padding: 3rem 2rem; display: flex; flex-direction: column; align-items: center; gap: 1rem; text-align: center; }
.fca-icon  { width: 72px; height: 72px; background: rgba(96,165,250,.1); border-radius: 20px; display: flex; align-items: center; justify-content: center; }
.fca-title { font-size: 1.5rem; font-weight: 800; color: var(--text-primary); margin: 0; }
.fca-sub   { font-size: .95rem; color: var(--text-secondary); max-width: 520px; line-height: 1.6; margin: 0; }
.fca-actions { margin-top: .5rem; }

/* ══════════════════════════════════════════
   RESPONSIVE
══════════════════════════════════════════ */
@media (max-width: 1024px) {
  .cmd-showcase { grid-template-columns: repeat(2, 1fr); }
  .qos-cards    { grid-template-columns: repeat(2, 1fr); }
  .summary-grid { grid-template-columns: repeat(2, 1fr); }
}
@media (max-width: 900px) {
  .arch-grid    { grid-template-columns: 1fr; }
  .exec-grid    { grid-template-columns: 1fr; }
  .perf-cards   { grid-template-columns: 1fr; }
  .discovery-grid { grid-template-columns: 1fr; }
  .domain-visual { grid-template-columns: 1fr; }
  .cbg-grid     { grid-template-columns: 1fr; }
  .pb-row { grid-template-columns: 160px 1fr 80px; }
}
@media (max-width: 768px) {
  .cmd-showcase { grid-template-columns: 1fr; }
  .qos-cards    { grid-template-columns: 1fr; }
  .lcm-track    { flex-direction: column; }
  .bt-cell      { padding: 8px 10px; font-size: .75rem; }
  .pb-row { grid-template-columns: 1fr 60px; }
  .pbr-label { display: none; }
}
@media (max-width: 480px) {
  .summary-grid { grid-template-columns: 1fr; }
}
</style>
