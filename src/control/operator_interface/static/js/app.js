const socket = io();

// Mission controls and status are handled by the Mission Supervisor.

// GPS Mission Control
const missionLatitude = document.getElementById('mission-latitude');
const missionLongitude = document.getElementById('mission-longitude');
const missionState = document.getElementById('mission-state');
const missionMessage = document.getElementById('mission-message');
const sendMissionButton = document.getElementById('send-mission-destination');
const confirmDeliveryButton = document.getElementById('confirm-delivery');
const cancelMissionButton = document.getElementById('cancel-mission');

sendMissionButton.addEventListener('click', () => {
    const latitude = Number(missionLatitude.value);
    const longitude = Number(missionLongitude.value);

    if (
        missionLatitude.value.trim() === '' ||
        missionLongitude.value.trim() === '' ||
        !Number.isFinite(latitude) ||
        !Number.isFinite(longitude) ||
        latitude < -90 || latitude > 90 ||
        longitude < -180 || longitude > 180
    ) {
        missionMessage.textContent = 'Invalid GPS coordinates.';
        return;
    }

    socket.emit('send_mission_destination', {
        latitude: latitude,
        longitude: longitude
    });

    missionMessage.textContent =
        'Destination submitted. Awaiting Mission Supervisor status.';
});

confirmDeliveryButton.addEventListener('click', () => {
    if (missionState.textContent !== 'ARRIVAL_CONFIRMATION') {
        return;
    }

    socket.emit('confirm_delivery');
    missionMessage.textContent = 'Delivery confirmation submitted.';
});

cancelMissionButton.addEventListener('click', () => {
    if (!['DELIVERING', 'RETURNING_HOME', 'ARRIVAL_CONFIRMATION'].includes(
        missionState.textContent
    )) return;
    if (!window.confirm('Request cancellation of the current mission?')) return;
    cancelMissionButton.disabled = true;
    socket.emit('cancel_mission');
    missionMessage.textContent =
        'Cancellation requested. Vehicle stop is NOT confirmed.';
});

socket.on('mission_status', (data) => {
    missionState.textContent = data.state;

    confirmDeliveryButton.disabled =
        data.state !== 'ARRIVAL_CONFIRMATION';
    cancelMissionButton.disabled = ![
        'DELIVERING', 'RETURNING_HOME', 'ARRIVAL_CONFIRMATION'
    ].includes(data.state);
});

socket.on('mission_message', (data) => {
    missionMessage.textContent = data.message;
});

socket.on('disconnect', () => {
    missionState.textContent = 'DISCONNECTED';
    confirmDeliveryButton.disabled = true;
    cancelMissionButton.disabled = true;
});

// Dashboard tab switching (display only)
const dashboardTabs = document.querySelectorAll('.dashboard-tabs .tab');

const tabPanels = {
    drive: document.querySelector('.drive-panel'),
    motor: document.querySelector('.motor-panel'),
    command: document.querySelector('.command-panel'),
    mission: document.querySelector('.mission-panel')
};

function showDashboardTab(selectedTab) {
    dashboardTabs.forEach(button => {
        const active = button.dataset.tab === selectedTab;
        button.classList.toggle('active', active);
        button.setAttribute('aria-pressed', String(active));
    });

    const visiblePanels = {
        overview: ['drive', 'motor', 'command', 'mission'],
        drive: ['drive', 'command'],
        motor: ['motor']
    };

    const panelsToShow = visiblePanels[selectedTab] || visiblePanels.overview;

    Object.entries(tabPanels).forEach(([name, panel]) => {
        if (panel) {
            panel.hidden = !panelsToShow.includes(name);
        }
    });
}

dashboardTabs.forEach(button => {
    button.addEventListener('click', () => {
        showDashboardTab(button.dataset.tab);
    });
});

showDashboardTab('overview');


// Forward real ROS 2 GPS telemetry to the offline map.
socket.on('gps_update', (data) => {
    window.dispatchEvent(new CustomEvent('saads:gps-update', {
        detail: data
    }));
});


// Remove live GPS visualization when the backend disconnects.
socket.on('disconnect', () => {
    window.dispatchEvent(new Event('saads:gps-disconnected'));
});

// Live GPS status monitoring
const gpsStatusElement = document.getElementById('gps-status');
let lastGpsStatusUpdate = 0;

function updateGpsStatus(status) {
    if (gpsStatusElement) {
        gpsStatusElement.textContent = status;
    }
}

window.addEventListener('saads:gps-update', (event) => {
    const gps = event.detail;
    lastGpsStatusUpdate = Date.now();

    if (gps && gps.valid === true) {
        updateGpsStatus('Valid Fix');
    } else {
        updateGpsStatus('No Fix');
    }
});

// Reset the GPS status when telemetry becomes stale.
setInterval(() => {
    if (
        lastGpsStatusUpdate === 0 ||
        Date.now() - lastGpsStatusUpdate > 5000
    ) {
        updateGpsStatus('No Data');
    }
}, 1000);

// Clear GPS status immediately if the backend disconnects.
window.addEventListener('saads:gps-disconnected', () => {
    lastGpsStatusUpdate = 0;
    updateGpsStatus('No Data');
});


// Forward real global EKF heading telemetry to the offline map.
socket.on('localization_update', (data) => {
    window.dispatchEvent(new CustomEvent('saads:localization-update', {
        detail: data
    }));
});


// Forward submitted GPS destinations to the offline map.
socket.on('destination_submitted', (data) => {
    window.dispatchEvent(new CustomEvent('saads:destination-submitted', {
        detail: data
    }));
});

// Track freshness of geographically converted Nav2 paths.
let lastPlannedPathUpdate = 0;

socket.on('planned_path_update', (data) => {
    lastPlannedPathUpdate = (
        data &&
        Array.isArray(data.coordinates) &&
        data.coordinates.length >= 2
    ) ? Date.now() : 0;

    window.dispatchEvent(new CustomEvent('saads:planned-path-update', {
        detail: data
    }));
});

// Clear the planned route when the operator server disconnects.
socket.on('disconnect', () => {
    window.dispatchEvent(new CustomEvent('saads:planned-path-update', {
        detail: {
            frame_id: 'wgs84',
            coordinates: [],
            pose_count: 0
        }
    }));
});
