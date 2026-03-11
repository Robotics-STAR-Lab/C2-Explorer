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

// Benchmark scene switcher
document.addEventListener('DOMContentLoaded', function () {
  const tabs = document.querySelectorAll('.bmk-tab');
  const videos = {
    ours:  document.getElementById('bmk-ours'),
    racer: document.getElementById('bmk-racer'),
    fame:  document.getElementById('bmk-fame'),
  };

  tabs.forEach(function (tab) {
    tab.addEventListener('click', function () {
      tabs.forEach(function (t) { t.classList.remove('is-active'); });
      tab.classList.add('is-active');

      const scene = tab.dataset.scene;
      const map = { ours: 'OURS', racer: 'RACER', fame: 'FAME' };

      Object.keys(videos).forEach(function (key) {
        const v = videos[key];
        const src = 'static/videos/bmk/' + map[key] + '_' + scene + '.mp4';
        v.querySelector('source').src = src;
        v.load();
        v.play();
      });
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
