(function () {
  if (window.Swal && typeof window.Swal.fire === 'function') return;

  var activeOverlay = null;

  function removeActive() {
    if (activeOverlay && activeOverlay.parentNode) {
      activeOverlay.parentNode.removeChild(activeOverlay);
    }
    activeOverlay = null;
  }

  function iconColor(icon) {
    if (icon === 'success') return '#16a34a';
    if (icon === 'warning') return '#f59e0b';
    if (icon === 'error') return '#dc2626';
    return '#2563eb';
  }

  function iconText(icon) {
    if (icon === 'success') return 'OK';
    if (icon === 'warning') return '!';
    if (icon === 'error') return 'X';
    return 'i';
  }

  function makeButton(text, primary) {
    var button = document.createElement('button');
    button.type = 'button';
    button.textContent = text;
    button.style.border = '0';
    button.style.borderRadius = '10px';
    button.style.padding = '10px 18px';
    button.style.fontSize = '14px';
    button.style.fontWeight = '700';
    button.style.cursor = 'pointer';
    button.style.minWidth = '110px';
    if (primary) {
      button.style.background = '#1d4ed8';
      button.style.color = '#fff';
    } else {
      button.style.background = '#e5e7eb';
      button.style.color = '#111827';
    }
    return button;
  }

  window.Swal = {
    fire: function (options) {
      options = options || {};
      removeActive();

      return new Promise(function (resolve) {
        var resolved = false;
        var timerId = null;

        function finish(result) {
          if (resolved) return;
          resolved = true;
          if (timerId) window.clearTimeout(timerId);
          removeActive();
          resolve(result || { isConfirmed: false, isDismissed: true });
        }

        var overlay = document.createElement('div');
        activeOverlay = overlay;
        overlay.style.position = 'fixed';
        overlay.style.inset = '0';
        overlay.style.zIndex = '2147483647';
        overlay.style.display = 'flex';
        overlay.style.alignItems = 'center';
        overlay.style.justifyContent = 'center';
        overlay.style.background = 'rgba(15, 23, 42, 0.42)';
        overlay.style.fontFamily = 'Arial, sans-serif';

        var modal = document.createElement('div');
        modal.style.width = 'min(440px, calc(100vw - 40px))';
        modal.style.borderRadius = '18px';
        modal.style.background = '#fff';
        modal.style.boxShadow = '0 24px 70px rgba(15, 23, 42, 0.35)';
        modal.style.padding = '28px';
        modal.style.textAlign = 'center';
        modal.style.color = '#111827';
        modal.style.transform = 'scale(0.96)';
        modal.style.opacity = '0';
        modal.style.transition = 'opacity 120ms ease, transform 120ms ease';

        var badge = document.createElement('div');
        var color = iconColor(options.icon);
        badge.textContent = iconText(options.icon);
        badge.style.width = '54px';
        badge.style.height = '54px';
        badge.style.margin = '0 auto 16px';
        badge.style.borderRadius = '999px';
        badge.style.display = 'flex';
        badge.style.alignItems = 'center';
        badge.style.justifyContent = 'center';
        badge.style.fontWeight = '800';
        badge.style.fontSize = '20px';
        badge.style.color = color;
        badge.style.border = '3px solid ' + color;

        var title = document.createElement('div');
        title.textContent = options.title || '';
        title.style.fontSize = '22px';
        title.style.fontWeight = '800';
        title.style.marginBottom = options.text ? '10px' : '18px';

        var text = document.createElement('div');
        text.textContent = options.text || '';
        text.style.fontSize = '15px';
        text.style.lineHeight = '1.45';
        text.style.color = '#374151';
        text.style.marginBottom = '22px';
        text.style.whiteSpace = 'pre-wrap';

        var actions = document.createElement('div');
        actions.style.display = options.showConfirmButton === false ? 'none' : 'flex';
        actions.style.gap = '10px';
        actions.style.justifyContent = 'center';
        actions.style.flexDirection = options.reverseButtons ? 'row-reverse' : 'row';

        var confirm = makeButton(options.confirmButtonText || 'OK', true);
        confirm.addEventListener('click', function () {
          finish({ isConfirmed: true, isDismissed: false });
        });

        var cancel = null;
        if (options.showCancelButton) {
          cancel = makeButton(options.cancelButtonText || 'Cancel', false);
          cancel.addEventListener('click', function () {
            finish({ isConfirmed: false, isDismissed: true });
          });
        }

        modal.appendChild(badge);
        if (options.title) modal.appendChild(title);
        if (options.text) modal.appendChild(text);
        if (cancel) actions.appendChild(cancel);
        actions.appendChild(confirm);
        modal.appendChild(actions);
        overlay.appendChild(modal);
        document.body.appendChild(overlay);

        window.setTimeout(function () {
          modal.style.opacity = '1';
          modal.style.transform = 'scale(1)';
        }, 0);

        overlay.addEventListener('click', function (event) {
          if (event.target === overlay && options.allowOutsideClick !== false) {
            finish({ isConfirmed: false, isDismissed: true });
          }
        });

        if (options.timer && !options.showCancelButton) {
          timerId = window.setTimeout(function () {
            finish({ isConfirmed: false, isDismissed: true, isTimer: true });
          }, Number(options.timer));
        }
      });
    }
  };
})();
