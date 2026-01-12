<template>
  <div class="action-policy-viz">
    <div class="viz-header">
      <div class="header-title">
        <q-icon name="dns" color="orange-4" />
        Server Goal Policy
      </div>
      <div class="header-control">
        <q-btn-toggle
          v-model="policy"
          :options="[
            { label: 'Reject New', value: 'reject' },
            { label: 'Abort Current', value: 'abort' },
            { label: 'Queue', value: 'queue' },
          ]"
          size="sm"
          toggle-color="orange"
          color="grey-9"
          spread
          no-caps
        />
      </div>
    </div>

    <!-- SERVER VISUALIZATION -->
    <div class="server-container">
      <div class="server-box">
        <div class="server-label">Action Server</div>

        <!-- ACTIVE SLOT -->
        <div class="slot active-slot">
          <div class="slot-label">Executing (Thread 1)</div>
          <transition name="pop">
            <div v-if="activeGoal" class="goal-agent active" :key="activeGoal.id">
              G{{ activeGoal.id }}
              <div class="progress-bar">
                <div class="fill" :style="{ width: activeGoal.progress + '%' }"></div>
              </div>
            </div>
            <div v-else class="empty-slot">IDLE</div>
          </transition>
        </div>

        <!-- QUEUE SLOT -->
        <div class="slot queue-slot" :class="{ disabled: policy !== 'queue' }">
          <div class="slot-label">Queue (Waitlist)</div>
          <transition-group name="list" tag="div" class="queue-list">
            <div v-for="goal in goalQueue" :key="goal.id" class="goal-agent queued">
              G{{ goal.id }}
            </div>
          </transition-group>
        </div>
      </div>

      <!-- INCOMING AREA -->
      <div class="incoming-area">
        <q-btn round color="blue" icon="add" size="lg" @click="sendGoal" :disable="cooldown">
          <q-tooltip>Send New Goal</q-tooltip>
        </q-btn>
      </div>
    </div>

    <!-- LOGS -->
    <div class="viz-log">
      <span v-if="lastLog" :class="lastLog.type">{{ lastLog.msg }}</span>
      <span v-else class="text-grey-7">System ready...</span>
    </div>
  </div>
</template>

<script setup lang="ts">
import { ref, onUnmounted } from 'vue';

type Policy = 'reject' | 'abort' | 'queue';
interface Goal {
  id: number;
  progress: number;
}

const policy = ref<Policy>('reject');
const activeGoal = ref<Goal | null>(null);
const goalQueue = ref<Goal[]>([]);
const nextId = ref(1);
const cooldown = ref(false);
const lastLog = ref<{ msg: string; type: string } | null>(null);

const log = (msg: string, type = 'text-white') => {
  lastLog.value = { msg, type };
  setTimeout(() => {
    if (lastLog.value?.msg === msg) lastLog.value = null;
  }, 2000);
};

const processGoals = () => {
  if (activeGoal.value) {
    activeGoal.value.progress += 2; // Speed
    if (activeGoal.value.progress >= 100) {
      log(`Goal G${activeGoal.value.id} Succeeded`, 'text-green-4');
      activeGoal.value = null;

      // Pull from queue
      if (goalQueue.value.length > 0) {
        activeGoal.value = goalQueue.value.shift()!;
        log(`Goal G${activeGoal.value.id} Started from Queue`, 'text-blue-4');
      }
    }
  } else if (goalQueue.value.length > 0) {
    activeGoal.value = goalQueue.value.shift()!;
  }
};

const interval = window.setInterval(processGoals, 50);
onUnmounted(() => clearInterval(interval));

const sendGoal = () => {
  const newGoal: Goal = { id: nextId.value++, progress: 0 };

  if (!activeGoal.value) {
    // Server IDLE
    activeGoal.value = newGoal;
    log(`Goal G${newGoal.id} Accepted`, 'text-green-4');
    return;
  }

  // Server BUSY - Apply Policy
  if (policy.value === 'reject') {
    log(`Goal G${newGoal.id} Rejected (Server Busy)`, 'text-red-4');
    // Visual bounce effect could be added here
  } else if (policy.value === 'abort') {
    log(`Goal G${activeGoal.value.id} Aborted. G${newGoal.id} Accepted.`, 'text-orange-4');
    activeGoal.value = newGoal;
  } else if (policy.value === 'queue') {
    if (goalQueue.value.length < 5) {
      goalQueue.value.push(newGoal);
      log(`Goal G${newGoal.id} Queued`, 'text-blue-4');
    } else {
      log('Queue Full!', 'text-red-4');
    }
  }

  cooldown.value = true;
  setTimeout(() => (cooldown.value = false), 200);
};
</script>

<style scoped>
.action-policy-viz {
  background: rgba(15, 23, 42, 0.9);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  overflow: hidden;
  max-width: 600px;
  margin: 2rem auto;
  font-family: 'Inter', sans-serif;
}

.viz-header {
  padding: 1rem;
  background: rgba(0, 0, 0, 0.4);
  display: flex;
  justify-content: space-between;
  align-items: center;
  border-bottom: 1px solid rgba(255, 255, 255, 0.1);
  gap: 1rem;
}

.header-title {
  color: #f1f5f9;
  font-weight: 700;
  display: flex;
  align-items: center;
  gap: 0.5rem;
  white-space: nowrap;
}

.server-container {
  padding: 2rem;
  display: flex;
  align-items: center;
  gap: 2rem;
}

.server-box {
  flex: 1;
  background: rgba(30, 41, 59, 0.5);
  border: 2px solid #475569;
  border-radius: 8px;
  padding: 1rem;
  display: flex;
  flex-direction: column;
  gap: 1rem;
}

.server-label {
  font-size: 0.75rem;
  color: #94a3b8;
  font-weight: 700;
  text-transform: uppercase;
  text-align: center;
}

.slot {
  border: 1px dashed rgba(255, 255, 255, 0.1);
  padding: 0.5rem;
  border-radius: 6px;
  min-height: 60px;
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  position: relative;
}

.slot.active-slot {
  background: rgba(34, 197, 94, 0.05); /* slightly green */
  border-color: rgba(34, 197, 94, 0.2);
}

.slot.queue-slot {
  background: rgba(59, 130, 246, 0.05); /* slightly blue */
  border-color: rgba(59, 130, 246, 0.2);
}

.slot.disabled {
  opacity: 0.3;
  pointer-events: none;
}

.slot-label {
  position: absolute;
  top: 2px;
  left: 4px;
  font-size: 0.6rem;
  color: #64748b;
}

.goal-agent {
  width: 80%;
  padding: 0.5rem;
  background: #f59e0b;
  color: white;
  font-weight: bold;
  border-radius: 4px;
  text-align: center;
  box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
  position: relative;
  overflow: hidden;
}

.goal-agent.queued {
  background: #3b82f6;
  font-size: 0.8rem;
  padding: 0.25rem;
  margin-bottom: 0.25rem; /* For list transition */
}

.progress-bar {
  position: absolute;
  bottom: 0;
  left: 0;
  height: 4px;
  width: 100%;
  background: rgba(0, 0, 0, 0.2);
}

.fill {
  height: 100%;
  background: rgba(255, 255, 255, 0.8);
  transition: width 0.05s linear;
}

.empty-slot {
  color: #475569;
  font-style: italic;
  font-size: 0.8rem;
}

.incoming-area {
  display: flex;
  align-items: center;
  justify-content: center;
}

.viz-log {
  padding: 0.5rem 1rem;
  background: #0f172a;
  text-align: center;
  font-family: 'Fira Code', monospace;
  font-size: 0.8rem;
  height: 30px;
}

/* Transitions */
.pop-enter-active,
.pop-leave-active {
  transition: all 0.3s cubic-bezier(0.175, 0.885, 0.32, 1.275);
}
.pop-enter-from {
  opacity: 0;
  transform: scale(0.5);
}
.pop-leave-to {
  opacity: 0;
  transform: scale(1.5);
}

.list-enter-active,
.list-leave-active {
  transition: all 0.3s;
}
.list-enter-from,
.list-leave-to {
  opacity: 0;
  transform: translateX(20px);
}
</style>
