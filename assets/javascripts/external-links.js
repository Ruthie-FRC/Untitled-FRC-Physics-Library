(() => {
  const applyTargets = () => {
    const links = document.querySelectorAll("a[href]");

    for (const link of links) {
      const href = link.getAttribute("href");

      if (!href) {
        continue;
      }

      if (
        href.startsWith("#") ||
        href.startsWith("/") ||
        href.startsWith("./") ||
        href.startsWith("../")
      ) {
        continue;
      }

      let url;

      try {
        url = new URL(href, window.location.href);
      } catch {
        continue;
      }

      if (url.origin === window.location.origin) {
        continue;
      }

      link.target = "_blank";
      link.rel = "noopener noreferrer";
    }
  };

  const scheduleApply = () => {
    window.requestAnimationFrame(applyTargets);
  };

  if (document.readyState === "loading") {
    document.addEventListener("DOMContentLoaded", scheduleApply, { once: true });
  } else {
    scheduleApply();
  }

  const observer = new MutationObserver(() => scheduleApply());
  observer.observe(document.documentElement, { childList: true, subtree: true });
})();
