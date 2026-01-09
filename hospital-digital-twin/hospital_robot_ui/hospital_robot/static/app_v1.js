let patientsData = {};
let selectedPatientId = null;
let selectedSampleId = null;
let pickupQueue = [];
let selectedQueueIdx = null;

async function init() {
    await updateDataFromServer();
    renderPatients();
}

// 서버 데이터 동기화 (Polling)
async function updateDataFromServer() {
    try {
        const res = await fetch("/api/patients");
        patientsData = await res.json();
        
        // 현재 화면 갱신
        if (selectedPatientId) {
            renderSamples(patientsData[selectedPatientId].samples);
            if (selectedSampleId) {
                const sample = findSample(selectedSampleId);
                const resultBox = document.getElementById("sample-result");
                if (sample.status === "검사 완료") {
                    resultBox.innerText = `[${sample.id}] 검사 결과\n\n${sample.result}`;
                } else {
                    resultBox.innerText = `현재 상태: ${sample.status}\n결과 대기 중...`;
                }
            }
        }
    } catch (e) { console.error("데이터 갱신 실패", e); }
}

function renderPatients() {
    const list = document.getElementById("patient-list");
    list.innerHTML = Object.entries(patientsData).map(([id, p]) => `
        <li onclick="onPatientSelected('${id}')" id="p-${id}">
            <strong>${id}</strong> / ${p.name}
        </li>
    `).join('');
}

function onPatientSelected(id) {
    selectedPatientId = id;
    const p = patientsData[id];
    document.querySelectorAll('#patient-list li').forEach(el => el.classList.remove('selected'));
    document.getElementById(`p-${id}`).classList.add('selected');
    document.getElementById("patient-info").innerText = 
        `환자 ID : ${id}\n이름 : ${p.name}\n생년월일 : ${p.dob}\n병동 : ${p.ward}\n비고 : ${p.notes}`;
    renderSamples(p.samples);
}

function renderSamples(samples) {
    const list = document.getElementById("sample-list");
    list.innerHTML = samples.map(s => `
        <li onclick="onSampleSelected('${s.id}')" id="s-${s.id}" class="${selectedSampleId === s.id ? 'selected' : ''}">
            ${s.id} / ${s.type} [${s.status}]
        </li>
    `).join('');
}

function onSampleSelected(sid) {
    selectedSampleId = sid;
    const sample = findSample(sid);
    document.querySelectorAll('#sample-list li').forEach(el => el.classList.remove('selected'));
    const selectedEl = document.getElementById(`s-${sid}`);
    if (selectedEl) selectedEl.classList.add('selected');

    const resultBox = document.getElementById("sample-result");
    if (sample.status === "검사 완료") {
        resultBox.innerText = `[${sample.id}] 검사 결과\n\n${sample.result}`;
    } else {
        resultBox.innerText = `현재 상태: ${sample.status}\n결과 대기 중...`;
    }
}

function findSample(sid) {
    for (let pid in patientsData) {
        let s = patientsData[pid].samples.find(item => item.id === sid);
        if (s) return s;
    }
}

function addToQueue() {
    if (!selectedSampleId) return showOverlay("알림", "검체를 선택하세요.");
    const sample = findSample(selectedSampleId);
    if (pickupQueue.some(q => q.sample_id === selectedSampleId)) return showOverlay("중복", "이미 추가되었습니다.");
    if (sample.status !== "수거 대기") return showOverlay("불가", "수거 대기 상태만 가능합니다.");
    
    pickupQueue.push({
        sample_id: selectedSampleId,
        patient_id: selectedPatientId,
        pickup_from: document.getElementById("departure-cb").value,
        lab: document.getElementById("lab-cb").value
    });
    renderQueue();
}

function renderQueue() {
    const list = document.getElementById("queue-list");
    document.getElementById("queue-count").innerText = pickupQueue.length;
  
    list.innerHTML = pickupQueue.map((q, idx) => `
      <li onclick="selectedQueueIdx = ${idx}">
        ${q.sample_id}<br>
        <small>출발: ${q.pickup_from} → 도착: ${q.lab}</small>
      </li>
    `).join('');
  }
  

function removeFromQueue() {
    if (selectedQueueIdx !== null) {
        pickupQueue.splice(selectedQueueIdx, 1);
        selectedQueueIdx = null;
        renderQueue();
    }
}

async function sendBatch() {
    if (pickupQueue.length === 0) return showOverlay("알림", "대기열이 비어있습니다.");
    await fetch("/api/send_batch", {
        method: "POST",
        headers: {"Content-Type": "application/json"},
        body: JSON.stringify(pickupQueue)
    });
    showOverlay("성공", "이송을 시작합니다.");
    pickupQueue = [];
    renderQueue();
    await updateDataFromServer();
}

function showOverlay(title, text) {
    document.getElementById("overlay-title").innerText = title;
    document.getElementById("overlay-text").innerText = text;
    document.getElementById("overlay").classList.remove("hidden");
}

function hideOverlay() { document.getElementById("overlay").classList.add("hidden"); }

setInterval(updateDataFromServer, 3000); // 3초 주기 자동 갱신
window.onload = init;