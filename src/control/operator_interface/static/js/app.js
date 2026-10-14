const socket = io();

const goalXInput = document.getElementById('goal-x');
const goalYInput = document.getElementById('goal-y');

const sendGoalButton = document.getElementById('send-goal');
const cancelGoalButton = document.getElementById('cancel-goal');

const navigationStatus = document.getElementById('navigation-status');
const distanceToGoal = document.getElementById('distance-to-goal');
const navigationSpeed = document.getElementById('navigation-speed');


socket.on('connect', () => {
    navigationStatus.textContent = 'Connected to SAADS server.';
});


socket.on('disconnect', () => {
    navigationStatus.textContent = 'Disconnected from SAADS server.';
});


sendGoalButton.addEventListener('click', () => {
    const x = Number(goalXInput.value);
    const y = Number(goalYInput.value);

    if (!Number.isFinite(x) || !Number.isFinite(y)) {
        navigationStatus.textContent = 'Invalid navigation goal.';
        return;
    }

    navigationStatus.textContent = 'Sending navigation goal...';

    socket.emit('send_goal', {
        x: x,
        y: y
    });
});


cancelGoalButton.addEventListener('click', () => {
    navigationStatus.textContent = 'Requesting navigation cancellation...';

    socket.emit('cancel_goal');
});


socket.on('navigation_status', (data) => {
    navigationStatus.textContent = data.message;
});


socket.on('navigation_feedback', (data) => {
    distanceToGoal.textContent = Number(data.distance).toFixed(2);
    navigationSpeed.textContent = Number(data.speed).toFixed(2);
});
