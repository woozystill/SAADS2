import * as maplibregl from '../vendor/maplibre/maplibre-gl.mjs';

const protocol = new pmtiles.Protocol();
maplibregl.addProtocol("pmtiles", protocol.tile);

const map = new maplibregl.Map({
    container: 'offline-map',
    style: "/static/map_styles/saads_dark.json",
    center: [-111.0, 32.2],
    zoom: 11,
    maxZoom: 17,
    maxBounds: [
        [-111.25, 31.95],
        [-110.65, 32.50]
    ],
    attributionControl: false
});

// Offline GPS map distance scale
map.addControl(
    new maplibregl.ScaleControl({
	maxWidth: 150,
	unit: 'metric'
    }),
    'bottom-left'
);

// Ground Station map controls (display only)
document.getElementById('map-recenter')?.addEventListener('click', () => {
    if (vehicleMarker !== null && Date.now() - lastGpsUpdate <= 5000) {
        map.flyTo({
            center: vehicleMarker.getLngLat(),
            zoom: Math.max(map.getZoom(), 15),
            essential: true
        });
    } else {
        map.flyTo({
            center: [-111.0, 32.2],
            zoom: 11,
            essential: true
        });
    }
});

document.getElementById('map-zoom-in')?.addEventListener('click', () => {
    map.zoomIn();
});

document.getElementById('map-zoom-out')?.addEventListener('click', () => {
    map.zoomOut();
});

// Live SAADS GPS visualization (read-only)
let vehicleMarker = null;
let lastGpsUpdate = 0;

const vehicleMarkerElement = document.createElement('div');

vehicleMarkerElement.innerHTML = `
    <svg width="30" height="30" viewBox="0 0 30 30">
        <path
            d="M15 3 L25 26 L15 21 L5 26 Z"
            fill="#ffffff"
            stroke="#10243a"
            stroke-width="2"
            stroke-linejoin="round"
        />
    </svg>
`;

vehicleMarkerElement.style.cssText = `
    width: 30px;
    height: 30px;
    display: flex;
    align-items: center;
    justify-content: center;
    filter: drop-shadow(0 0 4px rgba(0, 0, 0, 0.7));
    cursor: pointer;
`;

function hideVehicleMarker() {
    if (vehicleMarker !== null) {
        vehicleMarker.remove();
        vehicleMarker = null;
    }
}

window.addEventListener('saads:gps-update', (event) => {
    const gps = event.detail;

    if (
        !gps ||
        gps.valid !== true ||
        !Number.isFinite(gps.latitude) ||
        !Number.isFinite(gps.longitude)
    ) {
        hideVehicleMarker();
        return;
    }

    lastGpsUpdate = Date.now();

    const position = [gps.longitude, gps.latitude];

    if (vehicleMarker === null) {
        vehicleMarker = new maplibregl.Marker({
            element: vehicleMarkerElement,
            anchor: 'center'
        })
            .setLngLat(position)
            .setPopup(
                new maplibregl.Popup({ offset: 20 })
                    .setText('SAADS — Live GPS Position')
            )
            .addTo(map);
    } else {
        vehicleMarker.setLngLat(position);
    }

    applyVehicleHeading();
});

// Live vehicle heading from the global EKF (read-only).
let latestHeading = null;
let lastHeadingUpdate = 0;

function applyVehicleHeading() {
    if (vehicleMarker === null) return;

    const headingIsFresh =
        latestHeading !== null &&
        Date.now() - lastHeadingUpdate <= 5000;

    vehicleMarker.setRotation(headingIsFresh ? latestHeading : 0);

    // Arrow indicates known heading; circle indicates position only.
    vehicleMarkerElement.innerHTML = headingIsFresh
        ? `<svg width="30" height="30" viewBox="0 0 30 30">
               <path d="M15 3 L25 26 L15 21 L5 26 Z"
                     fill="#ffffff" stroke="#10243a"
                     stroke-width="2" stroke-linejoin="round"/>
           </svg>`
        : `<svg width="30" height="30" viewBox="0 0 30 30">
               <circle cx="15" cy="15" r="9"
                       fill="#ffffff" stroke="#10243a"
                       stroke-width="3"/>
           </svg>`;

    vehicleMarkerElement.style.opacity = '1';
    vehicleMarkerElement.title = headingIsFresh
        ? `SAADS heading: ${latestHeading.toFixed(1)}°`
        : 'SAADS GPS position — heading unavailable';
}

window.addEventListener('saads:localization-update', (event) => {
    const data = event.detail;

    if (
        !data ||
        !Number.isFinite(data.heading) ||
        data.frame_id !== 'map'
    ) {
        latestHeading = null;
        applyVehicleHeading();
        return;
    }

    latestHeading = ((data.heading % 360) + 360) % 360;
    lastHeadingUpdate = Date.now();
    applyVehicleHeading();
});

setInterval(applyVehicleHeading, 1000);

// Remove the marker if GPS telemetry becomes stale.
setInterval(() => {
    if (
        vehicleMarker !== null &&
        Date.now() - lastGpsUpdate > 5000
    ) {
        hideVehicleMarker();
    }
}, 1000);

window.addEventListener('saads:gps-disconnected', () => {
    hideVehicleMarker();
    latestHeading = null;
    lastHeadingUpdate = 0;
    lastGpsUpdate = 0;
});

console.log('SAADS live GPS visualization initialized.');

// Mission destination waypoint (display only)
let destinationMarker = null;

window.addEventListener('saads:destination-submitted', (event) => {
    const destination = event.detail;

    if (!destination) return;

    const latitude = Number(destination.latitude);
    const longitude = Number(destination.longitude);

    if (
        !Number.isFinite(latitude) ||
        !Number.isFinite(longitude) ||
        latitude < -90 || latitude > 90 ||
        longitude < -180 || longitude > 180
    ) {
        console.warn('Invalid destination coordinates received.');
        return;
    }

    const position = [longitude, latitude];

    if (destinationMarker === null) {
        const waypointElement = document.createElement('div');

        waypointElement.style.cssText = `
            width: 14px;
            height: 14px;
            background: #2684ff;
            border: 2px solid #ffffff;
            border-radius: 50%;
            box-shadow: 0 0 0 4px rgba(38, 132, 255, 0.2);
            cursor: pointer;
        `;

        destinationMarker = new maplibregl.Marker({
            element: waypointElement,
            anchor: 'center'
        })
            .setLngLat(position)
            .addTo(map);
    }

    destinationMarker
        .setLngLat(position)
        .setPopup(
            new maplibregl.Popup({ offset: 18 }).setText(
                `Submitted destination: ${latitude.toFixed(6)}, ${longitude.toFixed(6)}`
            )
        );

    // Center the map on the submitted destination (display only).
    map.flyTo({
        center: position,
        essential: true
    });

    console.log('SAADS submitted destination displayed:', position);
});

// Actual GPS traveled route (read-only visualization)
const traveledCoordinates = [];

map.on('load', () => {
    map.addSource('saads-traveled-route', {
        type: 'geojson',
        data: {
            type: 'Feature',
            properties: {},
            geometry: {
                type: 'MultiLineString',
                coordinates: []
            }
        }
    });

    map.addLayer({
        id: 'saads-traveled-route-line',
        type: 'line',
        source: 'saads-traveled-route',
        paint: {
            'line-color': '#2684ff',
            'line-width': 4,
            'line-opacity': 0.9
        },
        layout: {
            'line-join': 'round',
            'line-cap': 'round'
        }
    });

    console.log('SAADS traveled-route layer initialized.');
});

// Record actual GPS travel without connecting interrupted segments.
let activeRouteSegment = null;
let lastRouteFixTime = 0;

function gpsDistanceMeters(a, b) {
    const rad = Math.PI / 180;
    const lat1 = a[1] * rad;
    const lat2 = b[1] * rad;
    const dLat = (b[1] - a[1]) * rad;
    const dLon = (b[0] - a[0]) * rad;

    const h = Math.sin(dLat / 2) ** 2 +
        Math.cos(lat1) * Math.cos(lat2) *
        Math.sin(dLon / 2) ** 2;

    return 6371000 * 2 * Math.asin(Math.sqrt(Math.min(1, h)));
}

function updateTraveledRoute() {
    const source = map.getSource('saads-traveled-route');
    if (!source) return;

    source.setData({
        type: 'Feature',
        properties: {},
        geometry: {
            type: 'MultiLineString',
            coordinates: traveledCoordinates.filter(segment => segment.length >= 2)
        }
    });
}

window.addEventListener('saads:gps-update', (event) => {
    const gps = event.detail;
    const now = Date.now();

    if (
        !gps ||
        gps.valid !== true ||
        !Number.isFinite(gps.latitude) ||
        !Number.isFinite(gps.longitude)
    ) {
        activeRouteSegment = null;
        lastRouteFixTime = 0;
        return;
    }

    const position = [gps.longitude, gps.latitude];

    if (
        activeRouteSegment === null ||
        now - lastRouteFixTime > 5000
    ) {
        activeRouteSegment = [position];
        traveledCoordinates.push(activeRouteSegment);
    } else {
        const previous = activeRouteSegment[activeRouteSegment.length - 1];
        const distance = gpsDistanceMeters(previous, position);

        if (distance > 100) {
            activeRouteSegment = [position];
            traveledCoordinates.push(activeRouteSegment);
        } else if (distance >= 2) {
            activeRouteSegment.push(position);
        }
    }

    lastRouteFixTime = now;
    updateTraveledRoute();
});

// Keep recorded history, but separate routes across disconnects.
window.addEventListener('saads:gps-disconnected', () => {
    activeRouteSegment = null;
    lastRouteFixTime = 0;
});

// Separate segments when GPS messages stop arriving.
setInterval(() => {
    if (lastRouteFixTime && Date.now() - lastRouteFixTime > 5000) {
        activeRouteSegment = null;
        lastRouteFixTime = 0;
    }
}, 1000);

// Nav2 planned route, converted from the ROS map frame to WGS84.
map.on('load', () => {
    map.addSource('saads-planned-path', {
        type: 'geojson',
        data: {
            type: 'Feature',
            properties: {},
            geometry: {
                type: 'LineString',
                coordinates: []
            }
        }
    });

    map.addLayer({
        id: 'saads-planned-path-line',
        type: 'line',
        source: 'saads-planned-path',
        paint: {
            'line-color': '#ffad42',
            'line-width': 4,
            'line-opacity': 0.95,
            'line-dasharray': [2, 2]
        },
        layout: {
            'line-join': 'round',
            'line-cap': 'round'
        }
    });

    console.log('SAADS planned-path layer initialized.');
});

// Display geographically converted Nav2 paths without issuing commands.
window.addEventListener('saads:planned-path-update', (event) => {
    const source = map.getSource('saads-planned-path');
    if (!source) return;

    const data = event.detail;
    let coordinates = [];

    if (
        data &&
        data.frame_id === 'wgs84' &&
        Array.isArray(data.coordinates) &&
        data.coordinates.every(point =>
            Array.isArray(point) &&
            point.length === 2 &&
            Number.isFinite(point[0]) &&
            Number.isFinite(point[1]) &&
            point[0] >= -180 && point[0] <= 180 &&
            point[1] >= -90 && point[1] <= 90
        ) &&
        data.coordinates.length >= 2
    ) {
        coordinates = data.coordinates;
    }

    source.setData({
        type: 'Feature',
        properties: {},
        geometry: {
            type: 'LineString',
            coordinates: coordinates
        }
    });

    console.log('SAADS planned path updated:', coordinates.length, 'points');
});
