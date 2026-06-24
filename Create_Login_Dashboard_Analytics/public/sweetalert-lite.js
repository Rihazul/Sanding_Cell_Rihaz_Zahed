(function () {
  if (window.Swal && typeof window.Swal.fire === 'function') return;

  var activeOverlay = null;
  var styleInjected = false;

  function injectStyle() {
    if (styleInjected) return;
    styleInjected = true;
    var style = document.createElement('style');
    style.textContent =
      '@keyframes swalLiteBackdropIn{from{opacity:0}to{opacity:1}}' +
      '@keyframes swalLiteCardIn{0%{opacity:0;transform:translateY(18px) scale(.94)}100%{opacity:1;transform:translateY(0) scale(1)}}' +
      '@keyframes swalLiteIconPop{0%{transform:scale(.72);opacity:.2}70%{transform:scale(1.08);opacity:1}100%{transform:scale(1);opacity:1}}';
    document.head.appendChild(style);
  }

  function removeActive() {
    if (activeOverlay && activeOverlay.parentNode) {
      activeOverlay.parentNode.removeChild(activeOverlay);
    }
    activeOverlay = null;
  }

  function palette(icon) {
    if (icon === 'success') return { main: '#16a34a', soft: '#dcfce7', pale: '#f0fdf4', ring: '#86efac', text: 'OK' };
    if (icon === 'warning') return { main: '#d97706', soft: '#fef3c7', pale: '#fffbeb', ring: '#fbbf24', text: '!' };
    if (icon === 'error') return { main: '#dc2626', soft: '#fee2e2', pale: '#fef2f2', ring: '#fca5a5', text: 'X' };
    return { main: '#2563eb', soft: '#dbeafe', pale: '#eff6ff', ring: '#93c5fd', text: 'i' };
  }

  function makeButton(text, primary, colors) {
    var button = document.createElement('button');
    button.type = 'button';
    button.textContent = text;
    button.style.border = primary ? '0' : '1px solid #cbd5e1';
    button.style.borderRadius = '12px';
    button.style.padding = '11px 20px';
    button.style.fontSize = '14px';
    button.style.fontWeight = '800';
    button.style.letterSpacing = '.01em';
    button.style.cursor = 'pointer';
    button.style.minWidth = '118px';
    button.style.transition = 'transform 120ms ease, box-shadow 120ms ease, filter 120ms ease';
    if (primary) {
      button.style.background = 'linear-gradient(135deg, ' + colors.main + ', #0f172a)';
      button.style.color = '#fff';
      button.style.boxShadow = '0 12px 28px rgba(15, 23, 42, .24)';
    } else {
      button.style.background = '#f8fafc';
      button.style.color = '#0f172a';
      button.style.boxShadow = '0 6px 18px rgba(15, 23, 42, .08)';
    }
    button.addEventListener('mouseenter', function () {
      button.style.transform = 'translateY(-1px)';
      button.style.filter = 'brightness(1.03)';
    });
    button.addEventListener('mouseleave', function () {
      button.style.transform = 'translateY(0)';
      button.style.filter = 'brightness(1)';
    });
    return button;
  }

  window.Swal = {
    fire: function (options) {
      options = options || {};
      injectStyle();
      removeActive();

      return new Promise(function (resolve) {
        var resolved = false;
        var timerId = null;
        var colors = palette(options.icon);

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
        overlay.style.padding = '24px';
        overlay.style.background = 'radial-gradient(circle at 50% 40%, rgba(255,255,255,.18), transparent 34%), rgba(15, 23, 42, .52)';
        overlay.style.backdropFilter = 'blur(3px)';
        overlay.style.fontFamily = 'Segoe UI, Tahoma, sans-serif';
        overlay.style.animation = 'swalLiteBackdropIn 140ms ease-out both';

        var modal = document.createElement('div');
        modal.style.position = 'relative';
        modal.style.width = 'min(460px, calc(100vw - 42px))';
        modal.style.borderRadius = '26px';
        modal.style.background = 'linear-gradient(180deg, #ffffff 0%, #f8fafc 100%)';
        modal.style.boxShadow = '0 34px 90px rgba(2, 6, 23, .38), 0 0 0 1px rgba(255,255,255,.72) inset';
        modal.style.padding = '34px 34px 30px';
        modal.style.textAlign = 'center';
        modal.style.color = '#0f172a';
        modal.style.overflow = 'hidden';
        modal.style.animation = 'swalLiteCardIn 160ms cubic-bezier(.2,.9,.25,1.1) both';

        var accent = document.createElement('div');
        accent.style.position = 'absolute';
        accent.style.left = '0';
        accent.style.right = '0';
        accent.style.top = '0';
        accent.style.height = '7px';
        accent.style.background = 'linear-gradient(90deg, ' + colors.ring + ', ' + colors.main + ', #0f172a)';
        modal.appendChild(accent);

        var halo = document.createElement('div');
        halo.style.width = '96px';
        halo.style.height = '96px';
        halo.style.margin = '8px auto 18px';
        halo.style.borderRadius = '999px';
        halo.style.display = 'flex';
        halo.style.alignItems = 'center';
        halo.style.justifyContent = 'center';
        halo.style.background = 'radial-gradient(circle, ' + colors.soft + ' 0%, ' + colors.pale + ' 62%, transparent 63%)';
        halo.style.boxShadow = '0 18px 40px rgba(15, 23, 42, .13)';

        var badge = document.createElement('div');
        badge.textContent = colors.text;
        badge.style.width = '66px';
        badge.style.height = '66px';
        badge.style.borderRadius = '999px';
        badge.style.display = 'flex';
        badge.style.alignItems = 'center';
        badge.style.justifyContent = 'center';
        badge.style.fontWeight = '900';
        badge.style.fontSize = options.icon === 'success' ? '18px' : '30px';
        badge.style.color = colors.main;
        badge.style.background = '#fff';
        badge.style.border = '4px solid ' + colors.main;
        badge.style.boxShadow = '0 10px 26px rgba(15, 23, 42, .14)';
        badge.style.animation = 'swalLiteIconPop 220ms ease-out 80ms both';
        halo.appendChild(badge);

        var title = document.createElement('div');
        title.textContent = options.title || '';
        title.style.fontSize = '24px';
        title.style.fontWeight = '900';
        title.style.letterSpacing = '-.02em';
        title.style.marginBottom = options.text ? '10px' : '20px';
        title.style.color = '#0f172a';

        var text = document.createElement('div');
        text.textContent = options.text || '';
        text.style.fontSize = '15px';
        text.style.lineHeight = '1.55';
        text.style.color = '#475569';
        text.style.margin = '0 auto 24px';
        text.style.maxWidth = '360px';
        text.style.whiteSpace = 'pre-wrap';

        var actions = document.createElement('div');
        actions.style.display = options.showConfirmButton === false ? 'none' : 'flex';
        actions.style.gap = '12px';
        actions.style.justifyContent = 'center';
        actions.style.flexWrap = 'wrap';
        actions.style.flexDirection = options.reverseButtons ? 'row-reverse' : 'row';

        var confirm = makeButton(options.confirmButtonText || 'OK', true, colors);
        confirm.addEventListener('click', function () {
          finish({ isConfirmed: true, isDismissed: false });
        });

        var cancel = null;
        if (options.showCancelButton) {
          cancel = makeButton(options.cancelButtonText || 'Cancel', false, colors);
          cancel.addEventListener('click', function () {
            finish({ isConfirmed: false, isDismissed: true });
          });
        }

        modal.appendChild(halo);
        if (options.title) modal.appendChild(title);
        if (options.text) modal.appendChild(text);
        if (cancel) actions.appendChild(cancel);
        actions.appendChild(confirm);
        modal.appendChild(actions);
        overlay.appendChild(modal);
        document.body.appendChild(overlay);

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
