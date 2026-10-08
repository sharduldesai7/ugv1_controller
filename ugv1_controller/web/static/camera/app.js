  let currentMode = 'edges';
  const modeLabels = {raw:'COLOUR', gray:'GREYSCALE', edges:'EDGES', overlay:'OVERLAY'};

  function setMode(mode) {
    fetch('/processed-feed/mode/' + mode)
      .then(r => r.json())
      .then(() => {
        currentMode = mode;
        document.getElementById('mode-tag').textContent = modeLabels[mode] || mode.toUpperCase();
        ['raw','gray','edges','overlay'].forEach(m => {
          document.getElementById('btn-' + m).classList.toggle('active', m === mode);
        });
        // Force reload of processed feed to clear stale frame
        setTimeout(() => {
          const img = document.getElementById('img-proc');
          img.src = '/processed-feed/feed/proc?' + Date.now();
        }, 300);
      });
  }
