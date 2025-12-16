// This is a placeholder for the live dashboard logic.
// A real implementation would use WebSockets or WebRTC to receive real-time data
// from the perception stack running on the humanoids.

function setupDashboard() {
    console.log('Setting up perception dashboard...');

    const feeds = document.querySelectorAll('.feed');

    feeds.forEach(feed => {
        const canvas = feed.querySelector('canvas');
        const info = feed.querySelector('.info');
        const ctx = canvas.getContext('2d');

        // Simulate receiving data
        setInterval(() => {
            // Clear canvas
            ctx.clearRect(0, 0, canvas.width, canvas.height);

            // Draw placeholder text
            ctx.fillStyle = '#999';
            ctx.font = '20px sans-serif';
            ctx.textAlign = 'center';
            ctx.fillText('No Signal', canvas.width / 2, canvas.height / 2);

            // Update info
            info.innerHTML = `<p>Status: Disconnected</p><p>Last update: ${new Date().toLocaleTimeString()}</p>`;
        }, 2000);
    });
}

window.onload = setupDashboard;
