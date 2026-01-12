<template>
  <div class="qos-simulator">
    <div class="sim-header">
      <div class="header-title">
        <q-icon name="tune" color="blue-4" />
        QoS Compatibility Lab
      </div>
      <div class="status-badge" :class="connectionStatus">
        {{ connectionStatus === 'compatible' ? 'Conexión Establecida' : 'Incompatible QoS' }}
      </div>
    </div>

    <div class="sim-stage">
      <!-- PUBLISHER NODE -->
      <div class="node-card publisher">
        <div class="node-header">
          <q-icon name="send" />
          <span>Publisher</span>
        </div>

        <div class="qos-controls">
          <div class="control-group">
            <label>Reliability</label>
            <q-btn-toggle
              v-model="pubReliability"
              toggle-color="blue"
              :options="[
                { label: 'Reliable', value: 'reliable' },
                { label: 'Best Effort', value: 'best_effort' },
              ]"
              size="sm"
              spread
              no-caps
            />
          </div>

          <div class="control-group">
            <label>Durability</label>
            <q-btn-toggle
              v-model="pubDurability"
              toggle-color="purple"
              :options="[
                { label: 'Transient Local', value: 'transient_local' },
                { label: 'Volatile', value: 'volatile' },
              ]"
              size="sm"
              spread
              no-caps
            />
          </div>
        </div>
      </div>

      <!-- CONNECTION PIPE -->
      <div class="connection-pipe">
        <div class="pipe-line" :class="connectionStatus"></div>
        <div class="packet-flow" v-if="connectionStatus === 'compatible'">
          <div class="packet" v-for="n in 3" :key="n"></div>
        </div>
        <div class="broken-icon" v-else>
          <q-icon name="link_off" size="2rem" color="red" />
        </div>
      </div>

      <!-- SUBSCRIBER NODE -->
      <div class="node-card subscriber">
        <div class="node-header">
          <q-icon name="download" />
          <span>Subscriber</span>
        </div>

        <div class="qos-controls">
          <div class="control-group">
            <label>Reliability</label>
            <q-btn-toggle
              v-model="subReliability"
              toggle-color="blue"
              :options="[
                { label: 'Reliable', value: 'reliable' },
                { label: 'Best Effort', value: 'best_effort' },
              ]"
              size="sm"
              spread
              no-caps
            />
            <div class="compatibility-hint" v-if="reliabilityIncompatible">
              Requires Pub to be Reliable
            </div>
          </div>

          <div class="control-group">
            <label>Durability</label>
            <q-btn-toggle
              v-model="subDurability"
              toggle-color="purple"
              :options="[
                { label: 'Transient Local', value: 'transient_local' },
                { label: 'Volatile', value: 'volatile' },
              ]"
              size="sm"
              spread
              no-caps
            />
            <div class="compatibility-hint" v-if="durabilityIncompatible">
              Requires Pub to be Transient Local
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- EXPLANATION -->
    <div class="sim-explanation">
      <div v-if="connectionStatus === 'compatible'">
        <div v-if="lateJoinerEffect" class="text-green-4">
          <q-icon name="history" />
          <strong>Late Joiner:</strong> Subscriber recibirá los últimos mensajes guardados al
          conectarse.
        </div>
        <div v-else class="text-blue-4">
          <q-icon name="check" />
          <strong>Live Stream:</strong> Subscriber recibe mensajes generados <em>después</em> de
          conectar.
        </div>
      </div>
      <div v-else class="text-red-4">
        <q-icon name="error" />
        <strong>Rx > Tx Rule Violated:</strong> No puedes pedir (Subscriber) garantías más fuertes
        que las que ofrece el proveedor (Publisher).
      </div>
    </div>
  </div>
</template>

<script setup lang="ts">
import { ref, computed } from 'vue';

type Reliability = 'reliable' | 'best_effort';
type Durability = 'transient_local' | 'volatile';

const pubReliability = ref<Reliability>('reliable');
const pubDurability = ref<Durability>('volatile');

const subReliability = ref<Reliability>('reliable');
const subDurability = ref<Durability>('volatile');

const reliabilityIncompatible = computed(() => {
  // Config: Pub=BestEffort, Sub=Reliable -> INCOMPATIBLE
  return pubReliability.value === 'best_effort' && subReliability.value === 'reliable';
});

const durabilityIncompatible = computed(() => {
  // Config: Pub=Volatile, Sub=TransientLocal -> INCOMPATIBLE
  return pubDurability.value === 'volatile' && subDurability.value === 'transient_local';
});

const connectionStatus = computed(() => {
  if (reliabilityIncompatible.value || durabilityIncompatible.value) return 'incompatible';
  return 'compatible';
});

const lateJoinerEffect = computed(() => {
  // Both must be Transient Local for history effect
  return pubDurability.value === 'transient_local' && subDurability.value === 'transient_local';
});
</script>

<style scoped>
.qos-simulator {
  background: rgba(15, 23, 42, 0.9);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  overflow: hidden;
  max-width: 700px;
  margin: 2rem auto;
  font-family: 'Inter', sans-serif;
}

.sim-header {
  padding: 1rem;
  background: rgba(0, 0, 0, 0.4);
  display: flex;
  justify-content: space-between;
  align-items: center;
  border-bottom: 1px solid rgba(255, 255, 255, 0.1);
}

.header-title {
  color: #f1f5f9;
  font-weight: 700;
  display: flex;
  align-items: center;
  gap: 0.5rem;
}

.status-badge {
  padding: 0.25rem 0.75rem;
  border-radius: 20px;
  font-size: 0.8rem;
  font-weight: 600;
}

.status-badge.compatible {
  background: rgba(34, 197, 94, 0.2);
  color: #4ade80;
  border: 1px solid #22c55e;
}

.status-badge.incompatible {
  background: rgba(239, 68, 68, 0.2);
  color: #f87171;
  border: 1px solid #ef4444;
}

.sim-stage {
  padding: 2rem;
  display: flex;
  align-items: center;
  justify-content: space-between;
  gap: 1rem;
}

.node-card {
  flex: 1;
  background: rgba(30, 41, 59, 0.5);
  border-radius: 8px;
  padding: 1rem;
  border: 1px solid rgba(148, 163, 184, 0.2);
}

.node-header {
  display: flex;
  align-items: center;
  gap: 0.5rem;
  margin-bottom: 1rem;
  font-weight: 700;
  color: #e2e8f0;
  border-bottom: 1px solid rgba(255, 255, 255, 0.05);
  padding-bottom: 0.5rem;
}

.publisher .node-header {
  color: #60a5fa;
}
.subscriber .node-header {
  color: #a855f7;
}

.qos-controls {
  display: flex;
  flex-direction: column;
  gap: 1rem;
}

.control-group label {
  display: block;
  font-size: 0.75rem;
  color: #94a3b8;
  margin-bottom: 0.25rem;
  text-transform: uppercase;
}

.compatibility-hint {
  font-size: 0.7rem;
  color: #f87171;
  margin-top: 0.25rem;
}

/* CONNECTION PIPE */
.connection-pipe {
  position: relative;
  width: 100px;
  height: 40px;
  display: flex;
  align-items: center;
  justify-content: center;
}

.pipe-line {
  position: absolute;
  top: 50%;
  left: 0;
  width: 100%;
  height: 4px;
  background: #334155;
  transform: translateY(-50%);
  border-radius: 2px;
}

.pipe-line.compatible {
  background: #22c55e;
  box-shadow: 0 0 10px rgba(34, 197, 94, 0.5);
}
.pipe-line.incompatible {
  background: #ef4444;
  opacity: 0.3;
}

.packet-flow {
  position: absolute;
  width: 100%;
  height: 100%;
}

.packet {
  position: absolute;
  top: 50%;
  left: 0;
  width: 8px;
  height: 8px;
  background: #fff;
  border-radius: 50%;
  transform: translateY(-50%);
  animation: flow 1.5s linear infinite;
}

.packet:nth-child(2) {
  animation-delay: 0.5s;
}
.packet:nth-child(3) {
  animation-delay: 1s;
}

@keyframes flow {
  0% {
    left: 0;
    opacity: 1;
  }
  100% {
    left: 100%;
    opacity: 0;
  }
}

.broken-icon {
  z-index: 10;
  background: rgba(15, 23, 42, 0.8);
  border-radius: 50%;
}

.sim-explanation {
  padding: 1rem 1.5rem;
  background: rgba(0, 0, 0, 0.2);
  min-height: 60px;
  font-size: 0.9rem;
  display: flex;
  align-items: center;
  justify-content: center;
  text-align: center;
}
</style>
