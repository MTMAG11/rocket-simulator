"""The text of the Help panel. One entry per topic: (anchor, title, HTML). No Qt here."""

from __future__ import annotations

TOPICS: list[tuple[str, str, str]] = [
    (
        "start",
        "Start here: three steps",
        """
        <p>This program <b>pretends to launch a rocket</b> on your computer. It works out how the rocket flies using
        physics, then shows you what happened. Nothing real is launched.</p>
        <ol>
          <li><b>Pick a rocket.</b> (Box 1, top left.)</li>
          <li><b>Pick an engine.</b> (Box 2.) The rocket comes with an engine already chosen, so you can skip this at first.</li>
          <li>Press the big blue <b>Launch!</b> button.</li>
        </ol>
        <p>After a moment the answer appears: the numbers on top (how high, how fast), the story of the flight, charts, and a
        3-D picture you can watch and spin. The <b>Play</b> button at the bottom replays the flight.</p>
        <p>Then change one thing (a different engine, the wind) and launch again to see what changes. That is the whole game.</p>
        """,
    ),
    (
        "rocket",
        "1. Pick a rocket",
        """
        <p>The list holds ready-made rockets. Under the name you see a sentence about it and its size.</p>
        <ul>
          <li><b>Rockets</b> are the ones to start with. Small ones fly low; big ones fly high.</li>
          <li><b>Advanced experiments</b> show special things (a steering engine, a rocket built from separate parts). They are
          harder to understand and one of them is supposed to tumble.</li>
          <li><b>Browse...</b> opens a rocket file you made or got from somebody else.</li>
        </ul>
        <p>The rockets are <b>made-up examples</b>, not real kits, so the heights are only a pretend answer.</p>
        <p>Things that change the flight: a <b>heavier</b> rocket goes less high; a <b>fatter</b> rocket pushes more air out of
        the way and slows down faster; <b>fins</b> at the back keep it pointing straight (too small and it wobbles).</p>
        """,
    ),
    (
        "engine",
        "2. Pick an engine",
        """
        <p>The engine is what pushes the rocket up. The letter is how strong it is, and <b>each letter is about twice as
        strong as the one before</b>: a G engine pushes about twice as much as an F.</p>
        <table cellpadding="3">
          <tr><td><b>A, B</b></td><td>tiny</td><td><b>C, D</b></td><td>small</td></tr>
          <tr><td><b>E, F</b></td><td>medium</td><td><b>G, H</b></td><td>big</td></tr>
          <tr><td><b>I, J</b></td><td>bigger</td><td><b>K, L</b></td><td>huge</td></tr>
          <tr><td><b>M, N, O</b></td><td colspan="3">giant</td></tr>
        </table>
        <p>Under the box the program tells you how long it pushes (in seconds) and the total push, written
        <b>N&middot;s</b> (newton-seconds: a strength times a time).</p>
        <p><b>Does it fit?</b> The small line under the engine tells you if the engine fits inside the rocket and is strong
        enough to lift it. <span style="color:#6adf8a">Green</span> is fine, <span style="color:#f2c94c">yellow</span> works but
        not well, <span style="color:#ff6b6b">red</span> will not work (too wide, too long or too weak).</p>
        <p><b>Find more engines...</b> searches a big public list (ThrustCurve.org) and saves engines to your computer.
        You need the internet for that part only.</p>
        """,
    ),
    (
        "weather",
        "3. Launch angle and wind",
        """
        <ul>
          <li><b>Launch angle:</b> how steeply the rocket starts. <b>90 degrees is straight up</b>; 45 degrees is halfway
          between up and sideways. Lower angles fly lower and land farther away.</li>
          <li><b>Wind speed:</b> how hard the wind blows, in metres per second (1 m/s is a gentle breeze; 10 m/s is a strong
          wind). Wind makes a rocket lean into it and drift when it comes down on its parachute.</li>
          <li><b>Wind comes from:</b> the direction the wind blows <i>from</i> on a compass: 0 = north, 90 = east,
          180 = south, 270 = west. With wind from the west (270) the rocket drifts east.</li>
        </ul>
        """,
    ),
    (
        "steering",
        "4. Steering with the engine (TVC)",
        """
        <p>Some rockets can <b>steer with their engine</b>. This is called thrust vector control, or <b>TVC</b>: the engine
        nozzle tilts a little, which turns the rocket the way a rudder turns a boat. A small computer decides how far to
        tilt, many times a second, using sensors that feel how the rocket is moving.</p>
        <p>Tick <b>Steer with the engine nozzle</b> in box 4 to switch it on. The settings are:</p>
        <dl>
          <dt><b>Most it can tilt</b></dt><dd>How far the nozzle can swing to each side, in degrees. Real steering engines
          tilt only a few degrees.</dd>
          <dt><b>How fast it tilts</b></dt><dd>The quickest the nozzle can move, in degrees per second.</dd>
          <dt><b>Steering strength</b></dt><dd>How hard the computer pushes the rocket back toward where it should point.
          Too low and it reacts slowly; too high and it can overreact and shake.</dd>
          <dt><b>Calmness</b></dt><dd>How quickly it settles down after a correction. Low numbers swing back and forth; around
          0.7 to 1 settles smoothly.</dd>
          <dt><b>Trust the sensors / the true position</b></dt><dd>Real computers only have noisy sensors. The second choice
          gives the computer perfect knowledge, which is not possible on a real rocket.</dd>
        </dl>
        <p>Steering only works while the <b>engine is pushing</b>. After the engine stops there is nothing to tilt, so a
        rocket that is not balanced by itself will tumble. Turning steering on also turns the detail level up to 5 and adds a
        2 second wait on the pad while the sensors get ready. In the charts, the <b>Control</b> group shows what the computer
        asked for and what the nozzle really did.</p>
        <p>Try it on the <b>Steering-engine experiment</b> rocket, which is balanced badly on purpose: it tumbles without
        steering and flies straight with it.</p>
        """,
    ),
    (
        "launch",
        "Launch! and what happens",
        """
        <p>When you press <b>Launch!</b> the computer divides the flight into thousands of tiny moments. For each moment
        it works out the push of the engine, the pull of gravity, the push-back of the air and the wind, and moves the
        rocket a tiny bit. Put all the moments together and you have the whole flight.</p>
        <p>It usually takes a second or two. The same choices always give the same answer, because the small random
        wobbles are fixed by the <b>seed</b> number (see More settings).</p>
        <p>If something is wrong (the engine does not fit, or it is too weak to lift the rocket) a message tells you what
        to change.</p>
        """,
    ),
    (
        "results",
        "Your flight: the results box",
        """
        <p>Every launch is <b>saved automatically</b> in its own folder. The box under <b>Your flight</b> shows where.</p>
        <ul>
          <li><b>Open folder</b> opens that folder so you can look at the files (a table of numbers, a text summary and
          a picture).</li>
          <li><b>Export...</b> saves the flight data wherever you like, in a few file formats (for spreadsheets or
          programs).</li>
        </ul>
        """,
    ),
    (
        "tiles",
        "The big numbers on top",
        """
        <ul>
          <li><b>Highest point:</b> the greatest height above the ground.</li>
          <li><b>Top speed:</b> the fastest the rocket went.</li>
          <li><b>Engine pushed for:</b> how many seconds the engine burned.</li>
          <li><b>Flight time:</b> from launch until it touched the ground.</li>
          <li><b>Landed this far away:</b> how far from the launch pad it came down.</li>
        </ul>
        <p>Under each number it says <b>give or take</b>. The simulator is not perfect, so every number could really be a
        bit higher or lower. A star (*) means the guess is rougher than usual because this rocket is unlike the real
        ones the program was checked against.</p>
        """,
    ),
    (
        "story",
        "Start here tab: the story of your flight",
        """
        <p>After a launch, the <b>Start here</b> tab tells what happened in plain sentences: how high, how fast, when the
        parachute opened and how it landed. If the rocket had a problem (it wobbled, or it was only just stable) there is a
        note in yellow saying what went wrong.</p>
        """,
    ),
    (
        "charts",
        "Charts tab",
        """
        <p>Lines that show how things changed during the flight (time is along the bottom).</p>
        <ul>
          <li><b>Tick boxes on the left</b> choose what to draw: height, speed, push and many more. The names have units in
          brackets, like (m) for metres.</li>
          <li><b>Presets...</b> picks a ready-made set. <b>Flight overview</b> is a good one.</li>
          <li><b>Search channels</b> finds a name quickly.</li>
          <li><b>Time range</b> zooms in on the climb (<i>Until highest point</i>) or the engine burn (<i>Engine burn</i>).</li>
          <li>The <b>white line and dots</b> show where you are on the time bar at the bottom.</li>
          <li>The small tool icons above the chart zoom (magnifier) and move (cross) the picture; the house resets it.</li>
          <li>Dotted vertical lines mark events: leaving the rail, engine burnout, the highest point, the parachute and landing.</li>
        </ul>
        """,
    ),
    (
        "watch",
        "Watch it fly tab (3-D view)",
        """
        <p>A 3-D picture of the flight. The coloured line is the path; the brighter part is where the rocket has
        already been. The short white stick with a red tip is the rocket; an orange flame shows while the engine pushes.</p>
        <ul>
          <li><b>Left mouse drag:</b> turn the picture. <b>Right drag:</b> slide it. <b>Mouse wheel:</b> zoom.
          <b>Double-click:</b> back to the start view.</li>
          <li><b>Isometric / Side / Top:</b> ready-made viewpoints. <b>Follow rocket:</b> the camera keeps the rocket in the middle.</li>
          <li>The coloured key on the right tells which colour is which part of the flight (engine burning, coasting,
          parachute...). The green floor has a grid in metres. <b>E</b>, <b>N</b> and <b>Up</b> in the corner show which way
          is east, north and up. The blue stick is the wind.</li>
        </ul>
        """,
    ),
    (
        "play",
        "Play and the time bar",
        """
        <p>The bar at the bottom of the window is a <b>time machine</b> for the flight you just ran.</p>
        <ul>
          <li><b>Play</b> replays the flight. The speed box (1x, 5x, 20x, 50x) chooses how fast.</li>
          <li><b>Drag the slider</b> to jump to any moment. The charts and the 3-D picture follow.</li>
          <li>The line of words under it tells you the height, speed, engine push and weight at that moment. The line under
          that lists the main events with their times.</li>
        </ul>
        """,
    ),
    (
        "numbers",
        "Numbers tab",
        """
        <p>A table of results for grown-ups and scientists, with the warnings the simulator found. You do not need it to
        enjoy the game. Words like <i>static margin</i>, <i>Mach</i> and <i>apogee</i> are explained in the word list below.</p>
        """,
    ),
    (
        "advanced",
        "More settings (for experts)",
        """
        <p>Click <b>More settings</b> to open them. You can ignore all of these. They are for people who want to experiment.</p>
        <ul>
          <li><b>Empty weight:</b> the rocket without its engine, in kilograms. Heavier flies lower.</li>
          <li><b>Engine strength:</b> 1.0 is normal; 1.1 pushes 10 % harder than the engine really does.</li>
          <li><b>Launch direction:</b> the compass direction the rocket leans toward when the angle is less than 90 degrees.</li>
          <li><b>Air temperature change:</b> how many degrees warmer (+) or colder (-) than a standard day.</li>
          <li><b>Launch site height:</b> metres above sea level. Thinner air up high means less air resistance.</li>
          <li><b>Detail level:</b> how much the computer calculates. 3 is a good choice; higher numbers add sensors and
          a computer that steers the rocket (see 4), and take longer.</li>
          <li><b>Step size:</b> how big each tiny moment is. Smaller is more exact but slower.</li>
          <li><b>Method:</b> the maths used to step through time (rk4 is the careful one).</li>
          <li><b>Random seed:</b> a number that fixes the small random wobbles. The same seed gives exactly the same flight;
          change it for a slightly different one.</li>
        </ul>
        """,
    ),
    (
        "saved",
        "Saved runs tab (experts)",
        """
        <p>Opens big collections of flights made by the command-line tools (thousands of random flights for machine learning).
        You can look at them here. Ignore it unless you made one.</p>
        """,
    ),
    (
        "words",
        "Word list",
        """
        <dl>
          <dt><b>Apogee</b></dt><dd>The highest point of the flight.</dd>
          <dt><b>Thrust</b></dt><dd>The push of the engine, measured in newtons (N). One newton is about the weight of a
          small apple.</dd>
          <dt><b>Newton-second (N&middot;s)</b></dt><dd>Strength times time: the total push an engine gives. Bigger means a
          stronger engine.</dd>
          <dt><b>Engine class (A to O)</b></dt><dd>A letter for how strong the engine is. Each letter is about double the one
          before.</dd>
          <dt><b>Burnout</b></dt><dd>The moment the engine runs out and stops pushing. After that the rocket coasts.</dd>
          <dt><b>Drag</b></dt><dd>The air pushing back against the rocket. Faster means more drag.</dd>
          <dt><b>Fins</b></dt><dd>The small wings at the back that keep a rocket pointing the right way, like the feathers
          on an arrow.</dd>
          <dt><b>Stable</b></dt><dd>A rocket that points straight and fixes its own wobbles. It needs its balance point (centre
          of weight) in front of its centre of air push. The computer shows this as <b>static margin</b>, in &quot;calibers&quot;
          (body widths): above about 1 is steady, near 0 is touchy, below 0 will tumble.</dd>
          <dt><b>Mach</b></dt><dd>Speed compared with the speed of sound. Mach 1 is the speed of sound (about 340 m/s). The
          simulator is most trustworthy below Mach 0.9.</dd>
          <dt><b>Angle of attack</b></dt><dd>How much the rocket is turned sideways to the air it is flying through. 0 is flying
          nose-first.</dd>
          <dt><b>Parachute</b></dt><dd>Opens at the highest point so the rocket comes down gently.</dd>
          <dt><b>TVC (thrust vector control)</b></dt><dd>A steerable engine that tilts to keep the rocket pointing up. Only the
          advanced experiments use it.</dd>
          <dt><b>Seed</b></dt><dd>A number that fixes the random wobbles so a flight can be repeated exactly.</dd>
        </dl>
        """,
    ),
    (
        "problems",
        "If something does not work",
        """
        <ul>
          <li><b>A red box says the engine is too wide, too long or too weak:</b> pick a different engine or rocket.</li>
          <li><b>The rocket did not take off:</b> the engine is too weak. Pick a bigger-letter engine.</li>
          <li><b>The rocket tumbles or wobbles:</b> its fins are too small for its weight, or it is one of the advanced
          experiments (that one tumbles on purpose).</li>
          <li><b>Find more engines says it cannot reach the internet:</b> check your connection. The engines already on the
          list still work.</li>
          <li><b>Still stuck?</b> Look at the <b>Numbers</b> tab for warnings, or press F1 to open this help again.</li>
        </ul>
        """,
    ),
]

ANCHORS = [t[0] for t in TOPICS]


def html_all() -> str:
    """All topics as one page with anchors; headings use the same text as the topic list."""
    parts = []
    for anchor, title, body in TOPICS:
        parts.append(f'<a name="{anchor}"></a><h2>{title}</h2>{body}<hr>')
    return "<h1>Help</h1>" + "".join(parts)
