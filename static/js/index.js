window.HELP_IMPROVE_VIDEOJS = false;

// Copy BibTeX to clipboard
function copyBibTeX() {
    const bibtexElement = document.getElementById('bibtex-code');
    const button = document.querySelector('.copy-bibtex-btn');
    const copyText = button.querySelector('.copy-text');
    
    if (bibtexElement) {
        navigator.clipboard.writeText(bibtexElement.textContent).then(function() {
            // Success feedback
            button.classList.add('copied');
            copyText.textContent = 'Cop';
            
            setTimeout(function() {
                button.classList.remove('copied');
                copyText.textContent = 'Copy';
            }, 2000);
        }).catch(function(err) {
            console.error('Failed to copy: ', err);
            // Fallback for older browsers
            const textArea = document.createElement('textarea');
            textArea.value = bibtexElement.textContent;
            document.body.appendChild(textArea);
            textArea.select();
            document.execCommand('copy');
            document.body.removeChild(textArea);
            
            button.classList.add('copied');
            copyText.textContent = 'Cop';
            setTimeout(function() {
                button.classList.remove('copied');
                copyText.textContent = 'Copy';
            }, 2000);
        });
    }
}

// Benchmark performance data
var bmkData = {
  CUBICLE: {
    2: [
      { method: 'Ours',  tVal: '73.1 ± 4.9',   tPct: '↓42.9%', pVal: '253.1 ± 14.4',   pPct: '↓32.1%', vVal: '1.85 ± 0.01'                   },
      { method: 'RACER', tVal: '128.1 ± 24.1', tSecond: true,  pVal: '372.6 ± 57.9',   pSecond: true,  vVal: '1.84 ± 0.03', vSecond: true  },
      { method: 'FAME',  tVal: '142.6 ± 11.0',                  pVal: '473.8 ± 35.9',                   vVal: '1.71 ± 0.05'                   },
    ],
    3: [
      { method: 'Ours',  tVal: '58.8 ± 5.6',   tPct: '↓44.6%', pVal: '280.2 ± 11.4',   pPct: '↓31.3%', vVal: '1.84 ± 0.02'                   },
      { method: 'RACER', tVal: '106.2 ± 8.5',  tSecond: true,  pVal: '408.0 ± 43.3',   pSecond: true,  vVal: '1.84 ± 0.03', vSecond: true  },
      { method: 'FAME',  tVal: '115.9 ± 14.8',                  pVal: '560.5 ± 62.9',                   vVal: '1.70 ± 0.05'                   },
    ],
    4: [
      { method: 'Ours',  tVal: '50.9 ± 4.2',   tPct: '↓43.9%', pVal: '301.7 ± 14.9',   pPct: '↓32.3%', vVal: '1.82 ± 0.04'                   },
      { method: 'RACER', tVal: '90.7 ± 11.1',  tSecond: true,  pVal: '445.8 ± 74.7',   pSecond: true,  vVal: '1.80 ± 0.06', vSecond: true  },
      { method: 'FAME',  tVal: '100.8 ± 20.0',                  pVal: '614.1 ± 59.7',                   vVal: '1.74 ± 0.07'                   },
    ],
  },
  OPEN: {
    2: [
      { method: 'Ours',  tVal: '67.3 ± 6.7',   tPct: '↓46.5%', pVal: '214.5 ± 24.1',   pPct: '↓32.8%', vVal: '1.81 ± 0.04'                   },
      { method: 'RACER', tVal: '125.7 ± 28.9', tSecond: true,  pVal: '319.3 ± 54.0',   pSecond: true,  vVal: '1.78 ± 0.07', vSecond: true  },
      { method: 'FAME',  tVal: '169.9 ± 19.1',                  pVal: '553.9 ± 85.5',                   vVal: '1.67 ± 0.08'                   },
    ],
    3: [
      { method: 'Ours',  tVal: '53.8 ± 2.9',   tPct: '↓53.7%', pVal: '226.9 ± 9.0',    pPct: '↓33.8%', vVal: '1.81 ± 0.04'                   },
      { method: 'RACER', tVal: '116.1 ± 19.6', tSecond: true,  pVal: '342.8 ± 40.7',   pSecond: true,  vVal: '1.81 ± 0.05', vSecond: true  },
      { method: 'FAME',  tVal: '128.0 ± 24.4',                  pVal: '607.8 ± 107.6',                  vVal: '1.67 ± 0.05'                   },
    ],
    4: [
      { method: 'Ours',  tVal: '43.0 ± 2.0',   tPct: '↓51.8%', pVal: '243.9 ± 8.2',    pPct: '↓34.5%', vVal: '1.82 ± 0.06'                   },
      { method: 'RACER', tVal: '89.3 ± 27.5',  tSecond: true,  pVal: '372.2 ± 65.5',   pSecond: true,  vVal: '1.80 ± 0.03', vSecond: true  },
      { method: 'FAME',  tVal: '102.1 ± 13.3',                  pVal: '640.5 ± 65.3',                   vVal: '1.74 ± 0.05'                   },
    ],
  },
  MAZE: {
    2: [
      { method: 'Ours',  tVal: '89.5 ± 2.7',   tPct: '↓36.4%', pVal: '282.7 ± 21.4',   pPct: '↓32.7%', vVal: '1.70 ± 0.04'                   },
      { method: 'RACER', tVal: '140.7 ± 20.3', tSecond: true,  pVal: '420.2 ± 29.3',   pSecond: true,  vVal: '1.64 ± 0.07', vSecond: true  },
      { method: 'FAME',  tVal: '163.7 ± 15.8',                  pVal: '478.7 ± 45.1',                   vVal: '1.63 ± 0.02'                   },
    ],
    3: [
      { method: 'Ours',  tVal: '72.5 ± 3.8',   tPct: '↓31.5%', pVal: '304.6 ± 44.6',   pPct: '↓35.7%', vVal: '1.72 ± 0.02'                   },
      { method: 'RACER', tVal: '114.5 ± 21.5',                  pVal: '473.7 ± 52.4',   pSecond: true,  vVal: '1.71 ± 0.05', vSecond: true  },
      { method: 'FAME',  tVal: '105.8 ± 22.6', tSecond: true,   pVal: '508.2 ± 80.0',                   vVal: '1.65 ± 0.02'                   },
    ],
    4: [
      { method: 'Ours',  tVal: '52.8 ± 7.1',   tPct: '↓36.9%', pVal: '324.2 ± 32.9',   pPct: '↓34.5%', vVal: '1.70 ± 0.03'                   },
      { method: 'RACER', tVal: '83.7 ± 14.3',  tSecond: true,  pVal: '495.0 ± 53.6',   pSecond: true,  vVal: '1.69 ± 0.03', vSecond: true  },
      { method: 'FAME',  tVal: '96.3 ± 13.3',                   pVal: '586.9 ± 63.0',                   vVal: '1.67 ± 0.05'                   },
    ],
  },
};

function renderBmkTable(scene, drones) {
  var tbody = document.getElementById('bmk-table-body');
  if (!tbody) return;
  var rows = bmkData[scene][drones];

  function makeCell(val, isSecond, pct, isBest) {
    if (isBest) {
      return '<td><strong>' + val + '</strong>'
        + (pct ? '<span class="bmk-pct">' + pct + '</span>' : '')
        + '</td>';
    }
    return '<td><span class="' + (isSecond ? 'bmk-cell-second' : '') + '">' + val + '</span></td>';
  }

  tbody.innerHTML = rows.map(function(r) {
    var isOurs = r.method === 'Ours';
    var mClass = isOurs ? 'bmk-method-ours' : (r.method === 'RACER' ? 'bmk-method-racer' : 'bmk-method-fame');
    var rowClass = isOurs ? 'bmk-row-ours' : '';
    var methodHtml = isOurs ? '<strong>C<sup>2</sup>-Explorer</strong>' : r.method;
    return '<tr class="' + rowClass + '">'
      + '<td><span class="' + mClass + '">' + methodHtml + '</span></td>'
      + makeCell(r.tVal, r.tSecond, r.tPct, isOurs)
      + makeCell(r.pVal, r.pSecond, r.pPct, isOurs)
      + makeCell(r.vVal, r.vSecond, null,   isOurs)
      + '</tr>';
  }).join('');
}

// Benchmark scene switcher + drone selector + table
document.addEventListener('DOMContentLoaded', function () {
  var currentScene  = 'CUBICLE';
  var currentDrones = 2;

  renderBmkTable(currentScene, currentDrones);

  // Scene tabs
  var sceneTabs = document.querySelectorAll('.bmk-tab');
  var videos = {
    ours:  document.getElementById('bmk-ours'),
    racer: document.getElementById('bmk-racer'),
    fame:  document.getElementById('bmk-fame'),
  };

  sceneTabs.forEach(function (tab) {
    tab.addEventListener('click', function () {
      sceneTabs.forEach(function (t) { t.classList.remove('is-active'); });
      tab.classList.add('is-active');
      currentScene = tab.dataset.scene;

      var map = { ours: 'OURS', racer: 'RACER', fame: 'FAME' };
      Object.keys(videos).forEach(function (key) {
        var v = videos[key];
        var src = 'static/videos/bmk/' + map[key] + '_' + currentScene + '.mp4';
        v.querySelector('source').src = src;
        v.load();
        v.play();
      });

      renderBmkTable(currentScene, currentDrones);
    });
  });

  // Drone count tabs
  var droneTabs = document.querySelectorAll('.bmk-drone-tab');
  droneTabs.forEach(function (tab) {
    tab.addEventListener('click', function () {
      droneTabs.forEach(function (t) { t.classList.remove('is-active'); });
      tab.classList.add('is-active');
      currentDrones = parseInt(tab.dataset.drones, 10);
      renderBmkTable(currentScene, currentDrones);
    });
  });
});

// Motivation video modal
function openMotivVideo(src) {
  const modal = document.getElementById('motiv-video-modal');
  const source = document.getElementById('motiv-modal-source');
  const video = document.getElementById('motiv-modal-video');
  source.src = src;
  video.load();
  video.play();
  modal.classList.add('is-active');
  document.body.style.overflow = 'hidden';
}

function closeMotivVideo(event) {
  if (event && event.target !== event.currentTarget) return;
  const modal = document.getElementById('motiv-video-modal');
  const video = document.getElementById('motiv-modal-video');
  video.pause();
  modal.classList.remove('is-active');
  document.body.style.overflow = '';
}

document.addEventListener('keydown', function(e) {
  if (e.key === 'Escape') closeMotivVideo();
});

// Scroll to top functionality
function scrollToTop() {
    window.scrollTo({
        top: 0,
        behavior: 'smooth'
    });
}

// Show/hide scroll to top button
window.addEventListener('scroll', function() {
    const scrollButton = document.querySelector('.scroll-to-top');
    if (window.pageYOffset > 300) {
        scrollButton.classList.add('visible');
    } else {
        scrollButton.classList.remove('visible');
    }
});

// Video carousel autoplay when in view
function setupVideoCarouselAutoplay() {
    const carouselVideos = document.querySelectorAll('.results-carousel video');
    
    if (carouselVideos.length === 0) return;
    
    const observer = new IntersectionObserver((entries) => {
        entries.forEach(entry => {
            const video = entry.target;
            if (entry.isIntersecting) {
                // Video is in view, play it
                video.play().catch(e => {
                    // Autoplay failed, probably due to browser policy
                    console.log('Autoplay prevented:', e);
                });
            } else {
                // Video is out of view, pause it
                video.pause();
            }
        });
    }, {
        threshold: 0.5 // Trigger when 50% of the video is visible
    });
    
    carouselVideos.forEach(video => {
        observer.observe(video);
    });
}

// Teaser video carousel: switch slide only when current video ends
function setupTeaserVideoCarousel(teaserInstances) {
    if (!teaserInstances || teaserInstances.length === 0) return;

    const teaserCarousel = teaserInstances[0];
    const teaserRoot = document.querySelector('.teaser-video-carousel');
    if (!teaserRoot) return;

    const getCurrentVideo = () => teaserRoot.querySelector('.item.is-current video');

    teaserRoot.querySelectorAll('video').forEach(video => {
        video.addEventListener('ended', function() {
            teaserCarousel.next();
        });
    });

    teaserCarousel.on('show', function() {
        const currentVideo = getCurrentVideo();
        teaserRoot.querySelectorAll('video').forEach(video => {
            if (video !== currentVideo) {
                video.pause();
                video.currentTime = 0;
            }
        });
        if (currentVideo) {
            currentVideo.play().catch(() => {});
        }
    });

    const initialVideo = getCurrentVideo() || teaserRoot.querySelector('.item video');
    if (initialVideo) {
        initialVideo.play().catch(() => {});
    }

    // Replace pagination dots with text labels from data-label attributes
    const paginationContainer = teaserRoot.closest('.hero-body')
        ? teaserRoot.parentElement.querySelector('.slider-pagination')
        : teaserRoot.querySelector('.slider-pagination');
    if (!paginationContainer) return;

    const pages = paginationContainer.querySelectorAll('.slider-page');
    const items = teaserRoot.querySelectorAll('.item[data-label]');
    pages.forEach(function(page, i) {
        if (items[i] && items[i].dataset.label) {
            page.textContent = items[i].dataset.label;
            page.classList.add('slider-page-label');
            page.dataset.label = items[i].dataset.label;
        }
    });
}

// Keep hero button highlight for 5s after click, then restore.
function setupHeroButtonDelayHighlight() {
    const selector = '.hero-cover .publication-links .button.btn-arxiv, .hero-cover .publication-links .button.btn-code, .hero-cover .publication-links .button.btn-video';
    const buttons = document.querySelectorAll(selector);
    if (!buttons.length) return;

    buttons.forEach(button => {
        button.addEventListener('click', function() {
            button.classList.add('btn-delay-active');

            if (button._delayHighlightTimer) {
                clearTimeout(button._delayHighlightTimer);
            }

            button._delayHighlightTimer = setTimeout(function() {
                button.classList.remove('btn-delay-active');
                button._delayHighlightTimer = null;
            }, 3000);

            // Prevent focus style from sticking after mouse click.
            button.blur();
        });
    });
}

$(document).ready(function() {
    var baseOptions = {
        slidesToScroll: 1,
        slidesToShow: 1,
        loop: true,
        infinite: true
    };

    // Teaser carousel should not switch by timer.
    var teaserCarousels = bulmaCarousel.attach('.teaser-video-carousel', Object.assign({}, baseOptions, {
        autoplay: false
    }));

    // Other carousels can keep timed autoplay.
    bulmaCarousel.attach('.carousel:not(.teaser-video-carousel)', Object.assign({}, baseOptions, {
        autoplay: true,
        autoplaySpeed: 5000
    }));
		
    bulmaSlider.attach();
    
    // Setup video autoplay for videos in viewport
    setupVideoCarouselAutoplay();
    setupTeaserVideoCarousel(teaserCarousels);
    setupHeroButtonDelayHighlight();

})
