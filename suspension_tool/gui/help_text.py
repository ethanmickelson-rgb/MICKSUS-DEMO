"""The in-app user guide (Help menu). Kept as an embedded HTML string so
the packaged .exe needs no extra data files."""

HELP_HTML = """
<h1>MICKSUS — User Guide</h1>

<p><i>MICKSUS takes you from setup goals to hardpoints and back without
doing kinematics by hand. Everything solves live: change a number,
watch the 3D view and every curve react.</i></p>

<p><b>Contents:</b>
<a href="#start">Getting started</a> ·
<a href="#design">Designing a suspension (engineering guide)</a> ·
<a href="#new143">What's new in v1.43</a> ·
<a href="#new142">v1.42</a> ·
<a href="#new141">v1.41</a> ·
<a href="#new140">v1.40</a> ·
<a href="#new139">v1.39</a> ·
<a href="#new138">v1.38</a> ·
<a href="#new137">v1.37</a> ·
<a href="#new136">v1.36</a> ·
<a href="#new135">v1.35</a> ·
<a href="#new134">v1.34</a> ·
<a href="#new133">v1.33</a> ·
<a href="#new132">v1.32</a> ·
<a href="#new131">v1.31</a> ·
<a href="#new130">v1.30</a> ·
<a href="#new128">v1.28</a> ·
<a href="#new127">v1.27</a> ·
<a href="#new126">v1.26</a> ·
<a href="#new125">v1.25</a> ·
<a href="#new124">v1.24</a> ·
feature reference (below)</p>

<a name="new143"></a>
<h2>What's new in v1.43 (steering effort)</h2>
<ul>
<li><b>Sweeps ▸ Steering effort (rack force)…</b> gives the axial force in
the rack, the tie-rod force and the kingpin torque per wheel &mdash; the
numbers you size a bolted joint and a rack against &mdash; plus
steering-wheel torque and hand force at the rim.</li>
<li><b>The moment about the kingpin is computed in 3D</b>,
<i>k&middot;[(P&minus;A)&times;F]</i>, not the textbook
<i>Fy&times;trail</i>. That form assumes a vertical kingpin; yours leans
15.9&deg;, and the two differ by 4%.</li>
<li><b>The steering arm is the true moment arm</b>, not the scalar
"steering arm length" in the tweaks panel. Those differ by around 20% on a real
front (47.7 vs 57.1&nbsp;mm) because the tie rod is not perpendicular to
the kingpin &mdash; and the scalar errs on the light side, so using it
would under-size the joint.</li>
<li><b>Loads come from the Dynamics panel</b>, so they follow the car,
with a warning when the mass block is not plausible for the car being
modelled. Grip is load-sensitive, so the lightly loaded inside tire has
the HIGHER friction coefficient.</li>
<li><b>Verified against virtual work</b>: perturb the rack in the solver,
measure how far the knuckle turns about its kingpin, and check
<i>F&times;d(rack) = M&times;d(&theta;)</i>. Agreement 1.2e-4, and that
residual is steering jack, not error.</li>
<li><b>Two things it does NOT cover</b>, both making it an under-estimate:
pneumatic trail is zero by default (the aligning moment peaks BELOW the
limit &mdash; put a trail in to see that), and <b>parking effort is not
modelled and usually governs a manual rack</b>.</li>
</ul>

<a name="new142"></a>
<h2>What's new in v1.42 (live-tracking audit)</h2>
<ul>
<li>After the roll-angle track bug, every derived number was checked for
the same fault &mdash; a value computed from something you typed once
rather than from the model in front of you. <b>Three more were
found.</b></li>
<li><b>The roll and steer sweeps</b> both took track from the setup form,
exactly as the roll-angle readout did. They read the hardpoints now.</li>
<li><b>Front and rear track in the Dynamics panel are linked</b> (🔗).
They were typed by hand while the model already knew them, and track
divides straight into lateral load transfer, so a stale value skews the
whole balance. One real file was holding a 1/10 RC car's value &mdash;
left over from an earlier session on a very different vehicle.</li>
<li><b>Loaded tire radius is linked too</b>, taken from the contact
patch. Note the kinematics carry no tire squash, so this is the free
radius and reads a percent or two optimistic.</li>
<li><b>What is still typed by hand, on purpose:</b> vehicle wheelbase and
CG (each axle is modelled in its own local frame, so one corner's
hardpoints cannot supply them), masses, spring and damper rates, tire
friction data, and brake hardware. Those are real inputs, not derivable
&mdash; and they have their own plausibility checks.</li>
</ul>

<a name="new141"></a>
<h2>What's new in v1.41 (track vs overall width; live track)</h2>
<ul>
<li><b>New readout: "Overall width (tire edge to edge)".</b> Track width
is measured <b>mid-plane to mid-plane</b> (SAE J670 &mdash; the tool uses
the contact-patch centres). The distance across the OUTSIDES of the tires
is a whole tire width more, and CAD often quotes that one. On a real
file the two differed by exactly one tire width, which looks exactly like a
7&nbsp;in error in something until both numbers are on screen. Now they
are. Camber tilts the tread band and is ignored in the overall figure
(worth about 0.35&nbsp;in at 0.9&deg; on a 23&nbsp;in tire), because this
is the nominal dimension CAD quotes.</li>
<li><b>Fixed: roll angle used the seeded track, not the built one.</b>
Roll angle from travel is <i>atan(2&times;travel / track)</i>, and the
track it used came from the setup form &mdash; the value you ASKED for
when the axle was seeded. Any hardpoint edit moves the real track away
from it, and a batch entry of CAD coordinates can leave the two inches
apart. One real file had <b>a seed field a whole tire width off what the
linkage built</b>, biasing every roll number by 14.6%. It now comes from the
geometry.</li>
<li><b>New checklist row: "Built track matches the seeded intent".</b>
Flags when the setup form and the hardpoints disagree by more than 1% of
the car's own track, so the divergence above can never go quiet again.
The geometry always wins &mdash; re-fill the setup form from it.</li>
</ul>

<a name="new140"></a>
<h2>What's new in v1.40 (frame mesh travels with the file; sketch-yaw tweak)</h2>
<ul>
<li><b>The chassis mesh is saved INSIDE the project now.</b> Move a
design to another computer and the backdrop comes with it &mdash; no
hunting for the .3mf or .stl. The original file path is still recorded,
so you can re-link to a live CAD export if you want to. A 2.4&nbsp;MB
mesh adds about 3&nbsp;MB to the saved file; STL compresses much harder
than that. Autosave skips the embed (it runs on a timer, and a crash
recovery is always on the same machine anyway).</li>
<li><b>The chassis is drawn at 60% opacity</b>, up from 35%. Several
imported STL parts used to stack into a muddle, because transparent
surfaces are not depth-sorted. There is now an <b>Opacity</b> box in the
Frame panel so you can dial it, and it is saved with the project.</li>
<li><b>Sketch plane yaw is a tweak.</b> It used to be a seed input only,
so a sign error &mdash; typing +20&deg; for tubes that run &minus;20&deg;
&mdash; meant re-seeding. Now it is a spin box in Kinematic tweaks that
<b>re-squares the corner</b> onto the new plane, keeping bushing
midpoints, spread lengths and the kickup elevation. The recorded design
target follows automatically, so the checklist and the sketch-fit readout
grade against the new value.</li>
<li>The yaw row appears on the four types that actually have a 2D design
sketch: double wishbone, C-hub, H-arm and loaded halfshaft. It is applied
<i>before</i> the pin-skew rows, since squaring and skewing pull in
opposite directions and the skew should sit on top of the plane.</li>
</ul>

<a name="new139"></a>
<h2>What's new in v1.39 (batch hardpoint entry; derived-cell warning)</h2>
<ul>
<li><b>Batch edit.</b> Tick the box above the hardpoint table and nothing
solves until you press <b>Apply</b> &mdash; type the whole table, then
commit it in one go. Pending cells go bold amber and the button counts
them. <b>Revert</b> throws them away. Unticking the box applies whatever
is still pending.</li>
<li><b>A refused batch keeps your typing.</b> If the geometry will not
close, the cells stay exactly as entered and turn red, so you fix the one
that is wrong instead of losing the whole table. Live mode still reverts,
as it always has.</li>
<li><b>The reason an edit was refused now appears under the table</b>,
instead of only in a status bar that times out in the far corner of the
window. That silence was most of why a refused edit read as "the table
ignored me".</li>
<li><b>Derived coordinates now say so &mdash; for every point.</b> On a
double wishbone the knuckle convention keeps the spin axis in the
ball-joint plane and the steering arm at its set angle, which makes some
coordinates derived: you can type into them and they go straight back.
Measured on the example front, typing +10&nbsp;mm gives you
<b>0.00&nbsp;mm on wheel-centre X</b> and <b>0.13&nbsp;mm on tie-rod-outer
Y</b>. Only the wheel-centre case used to be reported, by name; the
tie-rod outer discarded 99% of what you typed without a word. The check is
now general, and it speaks only about cells <i>you</i> typed &mdash; other
points shifting is the convention doing its job.</li>
</ul>

<a name="new138"></a>
<h2>What's new in v1.38 (inputs commit on Enter; CAD-style CG marker)</h2>
<ul>
<li><b>Numeric fields no longer update as you type.</b> Every non-hardpoint
box &mdash; tweaks, vehicle, frame, shocks, dynamics, checklist targets,
travel range, rack limit &mdash; now waits for <b>Enter</b>, <b>Tab</b>, or
a click elsewhere before anything recomputes. Typing into a field showing
<i>45</i> used to walk the model through 451, 4512 and 45123 on the way to
your number, which is how a link ends up out at infinity.</li>
<li><b>Clicking into a field selects it.</b> Typing then <i>replaces</i>
the old number instead of appending to it &mdash; that append is the
actual cause of the runaway values. Click a second time inside the box if
you want to edit a single digit instead. <b>Esc</b> abandons an edit and
puts the committed value back.</li>
<li>The <b>arrows and the up/down stepper still act immediately</b>, since
nudging a value is the one place live feedback is wanted.</li>
<li><b>The seed and optimizer panels are unchanged</b> &mdash; they still
apply only when you press their button. They now also pick up a value you
typed but did not confirm, rather than reading the previous one.</li>
<li><b>The CG marker is now the CAD centre-of-mass symbol</b>: a ball
chequered into eight octants, gold and near-black, exactly as SolidWorks
draws it. Opposite octants share a colour, so from any viewpoint you see
four alternating quadrants &mdash; it reads as a position <i>and</i> an
orientation instead of a featureless blob. It is also <b>sized from your
car</b> now rather than being a fixed 22&nbsp;mm radius, so it stops
burying a 1/10 model.</li>
</ul>

<a name="new137"></a>
<h2>What's new in v1.37 (the dynamics panel goes metric)</h2>
<ul>
<li><b>The Dynamics panel follows the display-unit toggle.</b> Set the
toolbar to <b>mm</b> and it switches to SI throughout: <b>mass in kg</b>,
lengths in mm, forces and wheel loads in N, spring and tire rates in
N/mm, dampers in N&middot;s/mm, roll and ARB rates in N&middot;m/deg,
speed in km/h, CL&middot;A in m&sup2;. Set it back to <b>in</b> and you
get Milliken's imperial set exactly as before. The results list converts
too.</li>
<li><b>Weight becomes mass.</b> In metric the rows read &ldquo;Empty
mass&nbsp;(kg)&rdquo;, not &ldquo;Empty weight&rdquo; &mdash; the noun
changes with the unit, because kg is a mass and lb here is a weight. The
numbers are the same car either way (1&nbsp;lb =
0.45359237&nbsp;kg at one g).</li>
<li><b>Nothing else changes.</b> This is a display layer only. Saved
files, the Onshape push, the sweep dialogs and every formula stay
imperial internally, so a project written in metric opens identically in
imperial and vice versa &mdash; and <b>flipping the toggle cannot move a
value</b>, no matter how many times you flip it. The panel keeps its own
copy of every number rather than reading it back off the screen.</li>
<li><b>Metric precision is never coarser than imperial.</b> Each quantity
carries its own decimal count, chosen so an RC-scale number survives:
0.07&nbsp;kg of unsprung mass per corner and a 2.2&nbsp;N/mm spring both
keep their digits.</li>
<li><b>Two warnings were quietly scale-broken and are now relative.</b>
&ldquo;Inside corner loads near zero&rdquo; used a fixed 50&nbsp;lb
threshold, so it fired at every Ay on a 7&nbsp;lb car; it now reports the
lightest wheel as a <b>percentage of the car's weight</b>. The
front+rear axle sum check used a fixed 0.5&nbsp;lb tolerance, far too
loose at 1/10 scale; it is now a percentage too. Neither message quotes a
unit any more, so both read correctly in either system.</li>
</ul>

<a name="new136"></a>
<h2>What's new in v1.36 (three reported fixes)</h2>
<ul>
<li><b>The steering rack limit is saved now.</b> It used to reset to
38&nbsp;mm every time you reopened a design &mdash; it was the only UI
setting the file did not carry. Older files still open and keep the
default.</li>
<li><b>Toe link on the lower arm: the inner mount really does nothing, and
the panel now says so.</b> When the toe link's inner ball joint is bolted
to the same arm as the lower ball joint, it RIDES that arm &mdash; its
distance to the ball joint cannot change through travel. Moving it
&plusmn;30&nbsp;mm changes bump steer by under 0.05&nbsp;deg/mm, where a
chassis-mounted inner would change by more than 1.0. <b>Use the OUTER
rise instead</b>: it has about ten times the authority and crosses zero,
so you can actually dial bump steer out with it.</li>
<li><b>New: CG plausibility warning</b> in the Vehicle panel. The CG
height and CG-behind-front are typed by hand and are the easiest values
in the tool to leave at another car's numbers. A full-size CG on a 1/10
model puts the roll axis, the roll moment arm and every load-transfer
number into nonsense while everything else still looks fine. The check is
made against your car's own wheelbase, track and tire radius, so it works
at any scale.</li>
<li>If you were suspicious of the front roll centre: it is correct. It was
verified against an independent construction and agrees to
0.001&nbsp;mm.</li>
</ul>

<a name="new135"></a>
<h2>What's new in v1.35 (measured tire data, yaw-moment diagram)</h2>
<ul>
<li><b>Real measured tire data, at last.</b> A Dunlop KT821 22x8-10 tested
on hard clay and on gravel, shared publicly by an Auburn student on the
Baja SAE Discord. Peak grip and cornering stiffness now come from
measurement rather than from our own estimates, and both vary with load
the way a real tire does.</li>
<li><b>Our tire numbers moved, and not by a little.</b> Peak &mu; fell from
0.70 to 0.45 and cornering stiffness nearly doubled, from 16.5 to
30.2&nbsp;lb/deg. The old pair predicted 89&nbsp;lb of side force at a
150&nbsp;lb corner and 8&deg; of slip; measured says 66. <b>Expect lower
cornering numbers than before</b> &mdash; the car has not changed, our
estimate of the tire has.</li>
<li><b>The v1.29 tire-pressure worry is resolved.</b> Measured vertical
rate is 350&nbsp;lb/in at 10&nbsp;psi, which agrees with our estimate.
The racing rule of thumb that suggested we were 2&times; low does not
apply to this class of tire, and a Baja car's wheel hop stays near
9.8&nbsp;Hz.</li>
<li><b>New: Yaw-moment diagram</b> under <i>Sweeps</i>. This is the
Milliken Moment Method &mdash; a picture of the whole manoeuvring
envelope instead of a single operating point. It reports the lateral g
available, the g you can actually hold in a steady turn, a stability
index, control authority, and whether the car understeers or oversteers
<i>at the limit</i>, which is often the opposite of what it does in the
linear range.</li>
<li>Drop the speed in that dialog and watch it come apart: below about
18&nbsp;mph the control moment gain goes negative and the car resists
being steered. That is real, not a plotting artifact &mdash; at low speed
a given lateral acceleration demands a much higher yaw rate, which
saturates the rear tires on yaw-rate slip alone.</li>
<li><b>Honest caveat.</b> The measured tire is a Dunlop and ours is a
SunF; the author of the data suspects his model under-reports force, and
our own estimate ran optimistic. Call <code>tire_data_spread()</code> for
the band between them &mdash; at the design point it is about 36%%, so
treat absolute grip numbers as a range, not a value.</li>
</ul>

<a name="new134"></a>
<h2>What's new in v1.34 (C-hub squares up; the caster pill)</h2>
<ul>
<li><b>The C-hub now behaves like a double wishbone when you change the
kickup.</b> Its chassis bushing axis and its hinge pin are both meant to be
normal to the same 2D sketch, but the kickup tweak only moved the arm and
left the pin behind &mdash; 25&deg; of kickup left the corner 25&deg; out of
square. Both now move together, so the corner stays square.</li>
<li><b>Re-square works on the C-hub</b> (and the loaded halfshaft and
H-arm). It used to refuse anything but a double wishbone, which meant that
once the kickup had skewed a corner there was no way back. It squares the
arm and pin at the design yaw, keeps the kickup, and reports the before/after
misalignment in the status bar. The Design Checklist grades the same
number, so you can see when a corner has drifted.</li>
<li><b>New caster-pill row.</b> On the car an eccentric pill where the lower
arm's pin enters the C-block turns the block &mdash; and the kingpin fixed
in it &mdash; relative to the arm. The row works in degrees of caster added
on top of the chassis kickup, so <b>total caster = kickup + pill</b>, the
same way an RC setup sheet states it. Set 20&deg; of kickup and a
+2.5&deg; pill and you get 22.5&deg; of caster.</li>
<li><b>Re-square keeps your pill.</b> It repairs drift without undoing a
caster setting you chose.</li>
<li><b>v1.34.2 &mdash; the tweaks panel is grouped into sections</b>
(Shock mount, Steering, the type's own geometry, and <b>Placement</b>).
The C-hub list had grown to 21 rows, and the ride-height row sat at 19 of
21 where nobody could find it. Ride height, corner fore/aft and corner
up/down now sit together under <i>Placement</i> on every type &mdash;
they are the three that move the whole corner without changing any
kinematics.</li>
<li><b>v1.34.1 &mdash; the table now tells you when it cannot do what you
asked.</b> Three actions used to fail in total silence: an edit to the
wheel centre's fore/aft cell (that station is DERIVED on a double
wishbone &mdash; the knuckle convention keeps the spin axis in the
ball-joint plane, so use the ball joints or the corner fore/aft tweak to
move the axle), locking a halfshaft CV row (those are derived, not
hardpoints), and &mdash; worst &mdash; any internal error during a table
refresh, which used to leave the whole table dead until you restarted.
All three now explain themselves in the status bar, and a refresh error
can no longer disable the table. The <b>&perp; sketch column works for the
C-hub</b> now too.</li>
<li>Note that <i>Kingpin caster</i> and <i>Caster pill</i> are two routes to
the same angle &mdash; the first changes the block's built-in bore angle
(swapping the part), the second turns the eccentric. Edit one at a time;
whichever you touch wins.</li>
</ul>

<a name="new133"></a>
<h2>What's new in v1.33 (seed placement, axle rules, multilink shock)</h2>
<ul>
<li><b>"Initial seed placement X" is now the AXLE station.</b> The wheel
centre lands exactly on it, whatever the kickup and whatever the
suspension type. It used to anchor the inboard bushing axes, which let the
axle drift up to ~38&nbsp;mm as kickup changed &mdash; and since the rear
axle is exported shifted by the wheelbase you entered, two axles at
different kickup did not end up that far apart. Z still names the ground
plane, so both offsets now place the CAR.</li>
<li><b>Trailing arm is rear-axle only.</b> Measured: its toe is identical
at 0 and 20&nbsp;mm of rack &mdash; it has no tie rod, so a trailing-arm
front axle cannot steer. It joins the H-arm and loaded halfshaft. The
front axle offers double wishbone, multilink and C-hub.</li>
<li><b>The multilink seed respects your shock length and motion ratio.</b>
It used to inherit the double wishbone's shock re-homed to the lower ball
joint, landing 26% long with the wrong MR. Now it places the chassis eye
to hit both.</li>
<li><b>Steering arm fore/aft is a setup-form field.</b> It was fixed at
&minus;80&nbsp;mm, which is an 80&nbsp;mm steering arm on a 250&nbsp;mm
track at 1/10 scale.</li>
<li>The Design Checklist now explains why the toe-holding rear layouts
report ~0% anti-dive/anti-squat, instead of leaving it looking broken.</li>
</ul>

<a name="new132"></a>
<h2>What's new in v1.32 (seeding, axle rules, corner placement)</h2>
<ul>
<li><b>Generate now respects the suspension-type dropdown.</b> It used to
always build a double wishbone, so picking C-hub and pressing <i>Generate
seed hardpoints</i> quietly replaced it with a DW. Pick the type, press
Generate, get that type.</li>
<li><b>The rear-only layouts are no longer offered on the front axle.</b>
An H-arm and a loaded-halfshaft knuckle both hold toe rigidly by design
and have no steering input &mdash; a car with one on the front cannot
steer at all. They now appear on the rear axle only, and a forced request
(from a loaded project, say) is refused with an explanation rather than
silently accepted.</li>
<li><b>Move a whole corner without reseeding.</b> Two rows at the bottom of
the tweaks panel for every type: <i>Move corner fore/aft</i> and
<i>Move corner up/down</i>. Both are rigid &mdash; they translate every
point, so no curve changes. Use these instead of the ride-height row when
you just want to nudge the corner onto the frame: ride height is a target
measured from the ground up to the frame datum, so it moves the corner the
opposite way and depends on the tire radius.</li>
</ul>

<a name="new131"></a>
<h2>What's new in v1.31 (tweaks for every suspension type)</h2>
<ul>
<li><b>The other five types finally have real tweak sets.</b> The panel used
to show 19 rows for a double wishbone and as few as 2 for a multilink.
Now: double wishbone 19, <b>C-hub 19</b>, H-arm 12, loaded halfshaft 11,
trailing arm 8, multilink 5.</li>
<li><b>C-hub gets the steering knobs it was missing</b> — tie-rod outer and
inner rise (the bump-steer pair, the same motion the seed already
secant-tunes for you), steering-arm length, static toe and kingpin length.
Bump steer no longer needs hand-edited coordinates.</li>
<li><b>New shared rows</b>, offered wherever they mean something: installed
shock length @ ride, ride height, corner fore/aft, hub offset, arm
bushing-axis kickup, arm length, and <b>camber-link rises</b> on the C-hub
and H-arm. The camber link is the roll-centre control on those types and
had no tweak at all: 50&nbsp;mm of inner-ball travel is worth about
99&nbsp;mm of roll-centre height.</li>
<li><b>Hub offset appears only where there is a steering axis</b> (ball
joints, a physical kingpin, or an outer CV). A trailing-arm, multilink or
H-arm upright has none, so the row would read a meaningless 0.</li>
<li><b>Editing one row no longer nudges the others.</b> Every optional row
is skipped unless its spin actually moved, so the values shown at display
precision can't walk your geometry while you tune something else.</li>
<li>Two rows genuinely fight over the same point — steering-arm length vs
tie-rod outer rise, and <i>axle_x</i> vs hub offset. Edit them one at a
time and each lands exactly; the tooltips and the equation sheet say which
wins if both change at once.</li>
</ul>

<a name="new130"></a>
<h2>What's new in v1.30 (works at 1/10 RC scale)</h2>
<ul>
<li><b>Model-car dimensions can now be entered.</b> Every length range in the
setup form, the hardpoint scalars and the vehicle panel used to be floored at
full-size Baja values (800&nbsp;mm track, 1000&nbsp;mm wheelbase, 100&nbsp;mm
tire radius, 100&nbsp;mm shocks), so a 1/10 buggy simply could not be typed
in. The lower bounds now reach model-car size (~250&nbsp;mm track,
~285&nbsp;mm wheelbase, ~31&nbsp;mm tire radius, ~62&nbsp;mm shocks).
<b>Defaults are unchanged</b> — still a full-size Baja car.</li>
<li><b>Three hidden constants that would have broken RC geometry are gone.</b>
Opening the ranges alone was not enough: the C-hub tie-rod search band was a
fixed &plusmn;120&nbsp;mm (nearly half an RC wheelbase), the loaded-halfshaft
diff flange was pinned at y&nbsp;=&nbsp;120&nbsp;mm (out at the wheel on a
250&nbsp;mm track), and the articulation probe stepped in 1&nbsp;mm increments
(most of an RC car's travel in a single step). All three are now fractions of
the car, and reproduce their old values at Baja size to within 0.2% — your
existing seeds do not move.</li>
<li><b>Dynamics panel precision.</b> Weights carried 0 decimals, so a 1/10
buggy's 0.15&nbsp;lb unsprung corner rounded to <i>zero</i> and every ride
frequency, Bode plot and damping ratio downstream was garbage. Weights now
carry 2&ndash;3 decimals, dampers and cornering stiffness 2&ndash;3, and
front/rear track reach below 10&nbsp;in.</li>
<li><b>Scale checks in the test suite.</b> All five suspension types are
seeded and articulated at 1/10 scale on every run, with exact static
read-backs and the motion-ratio goal asserted — plus a guard that no seeded
hardpoint lands outside the car, which is the shape of bug this class of
fix exists for.</li>
<li><b>Chassis kickup now goes to 30&deg;</b> (was 20&deg;). RC buggies run
much more kickup than a Baja car. Note the interaction: with <i>independent
caster</i> OFF, a double wishbone's caster is slaved to kickup (30&deg; kickup
gives 30&deg; caster), which matches the RC rule that total caster = kick-up +
caster-block angle. The <b>C-hub is different</b> &mdash; its kingpin is a
physical pin, so <i>Desired caster</i> sets the TOTAL steering-axis lean
directly and does not add to kickup. Enter the number you actually want.</li>
<li>Reading the Bode plot for a 1/10 car: see <i>Help &rarr; Reading the Bode
plot</i>, which already covers the RC case (Froude scaling puts the target
ride frequency about 3.2&times; higher than full size).</li>
</ul>

<a name="new128"></a>
<h2>What's new in v1.28 (motion-ratio convention flipped)</h2>
<ul>
<li><b>Motion ratio is now SHOCK travel / WHEEL travel</b> — below 1, around
<b>0.55</b> for a Baja corner, instead of the old wheel/shock figure. This is
the convention the literature uses, so the number now drops straight into
<b>K<sub>w</sub> = K<sub>s</sub>·MR²</b> with no inversion.</li>
<li><b>Why it changed.</b> The tool always computed correctly internally —
there was a 1/MR conversion at the boundary between the kinematics and the
dynamics layer. But the number <i>displayed</i> was the wheel/shock one, and that is the
number a person carries into a textbook or an online calculator. Reading
a wheel/shock ratio into K<sub>w</sub> = K<sub>s</sub>·MR² squares the
mistake into a <b>several-fold error</b>. The
conversion bridge is now deleted rather than documented.</li>
<li><b>Quick sanity check:</b> because MR &lt; 1, the wheel rate is always
<i>lower</i> than the spring rate. If your hand calc comes out higher, the
ratio is upside down.</li>
<li><b>Your saved files still work.</b> Projects written before this change
are detected and converted automatically on load — the motion-ratio goal and
the optimizer target are inverted, and a dialog tells you exactly what
changed. The geometry itself is untouched; only the way the ratio is
written. Re-save to store it in the new form.</li>
<li><b>Also fixed:</b> the equation sheet defined MR as wheel/spring while
listing K<sub>w</sub> = K<sub>s</sub>·MR² — mutually inconsistent, and
exactly the trap this change removes. Corrected, along with the
virtual-work relation, which is now
F<sub>shock</sub> = F<sub>wheel</sub> / MR.</li>
</ul>

<a name="new127"></a>
<h2>What's new in v1.27 (equation reference)</h2>
<ul>
<li><b>Help ▸ Equations</b> (or <b>Shift+F1</b>) — every formula the tool
uses, in one scrollable window, so you can reproduce any number it reports by
hand. 82 equations across 15 sections: the kinematic solver core (Rodrigues,
circle∩sphere, trilateration, Kabsch), wheel angles, instant/roll centre,
motion ratio and rates, anti-geometry, mass and CG, load transfer,
frequencies and damping, steering and brakes, the tire model, CV joints, the
impact load path, and both frequency-response models.</li>
<li><b>Every entry names its symbols and the module that implements it</b>,
so you can go from a number on screen to the line of code that produced it.
There is also a symbol table and an explicit units warning up front — the
geometry layer is millimetres, the dynamics layer is inches/pounds/seconds,
so <b>mass = W / 386.4</b>. Using 32.2 there is the classic factor-of-12
error.</li>
<li><b>Printable version.</b> The window has an <i>Open printable PDF</i>
button (<code>suspension_tool/docs/Equation_Sheet.pdf</code>). Both the
window and the PDF render from the same source module, so they cannot
disagree; regenerate the PDF with
<code>python tools/build_equation_sheet.py</code>.</li>
<li><b>It is tested against the code, not just spell-checked.</b> Fourteen
tests evaluate the documented formula by hand and compare it against what the
software actually computes (load transfer, roll gradient, wheel/ride rate,
damping ratio, brake chain, impulse, wheel hop, the ARB conversion, …), and
another checks that every cross-referenced module still exists. A sheet that
quietly stops matching the code is worse than no sheet, because it is
trusted.</li>
</ul>

<a name="new126"></a>
<h2>What's new in v1.26 (whole-car Bode — 7 DOF)</h2>
<ul>
<li><b>A model selector in the Bode dialog.</b> Switch between the
quarter-car (one corner) and the <b>full car</b>: heave, pitch and roll of
the sprung mass plus all four unsprung masses, 7 degrees of freedom.</li>
<li><b>Wheelbase filtering — the reason to bother.</b> The rear wheels hit
the same bump the fronts did, delayed by L/V, so the response depends on
<b>speed</b>. At f&nbsp;=&nbsp;n·V/L the axles are in phase and only bounce
is excited (pitch has a null); at f&nbsp;=&nbsp;(2n+1)·V/2L they are opposed
and only pitch is excited (bounce has a null). That comb is drawn on the
pitch and heave panels, and it is why one set of whoops is brutal at one
speed and calm at another. A quarter-car model cannot show this at all.</li>
<li><b>The real coupled modes.</b> Bounce and pitch are not independent; the
dynamics sheet's <code>bounce</code>/<code>pitch</code> frequencies are the
classic <i>decoupled</i> formulas. The full car solves the coupled
eigenproblem and reports all seven modes, each <b>labelled from its
eigenvector</b> (heave / pitch / roll / wheel hop front &amp; rear) so you
know which is which.</li>
<li><b>Roll — and therefore the anti-roll bars.</b> A quarter car has no roll
DOF, so a bar is invisible to it. Here the bars enter the roll stiffness and
the roll mode is reported. (The road input drives both wheels of an axle
together, which by symmetry never excites roll — that is correct, and the
roll mode is still where you read what the bars are doing.)</li>
<li><b>Body acceleration at the axle stations, not just the CG</b> — pitch
means the front and rear of the car feel different things, which is what you
actually notice.</li>
<li><b>New input: roll radius of gyration</b> (Dynamics ▸ Mass &amp; CG),
defaulted to an estimate (~0.35×track). Pitch inertia still comes from
Olley's dynamic-index-1 assumption. <b>Both are estimates</b> and the report
says so — measure them if a decision turns on the pitch/roll split.</li>
</ul>

<a name="new125"></a>
<h2>What's new in v1.25 (ride/grip frequency response — the Bode plot)</h2>
<ul>
<li><b>Sweeps → Ride/grip frequency response (Bode).</b> The dynamics sheet
reports a ride frequency and a damping ratio, but those are <i>scalars</i>,
and the two things that decide how the car behaves on rough ground are
shapes. This plots the quarter-car frequency response: four channels — body
acceleration (harshness), <b>tire deflection (grip)</b>, body motion
(isolation) and suspension travel used — for both axles.</li>
<li><b>It shows the wheel-hop mode, which nothing else in the tool did.</b>
There are <i>two</i> resonances: the body mode (~2 Hz, the one the ride
frequency describes) and <b>wheel hop</b> (~10 Hz), where the unsprung mass
bounces on the tire spring. Wheel hop is what makes a tire skip and stop
putting power down, and no scalar can express it. Both are marked on every
plot (dotted = body, dashed = hop).</li>
<li><b>Terrain table — the actionable part.</b> Bumps spaced λ apart at speed
V drive a frequency f&nbsp;=&nbsp;V/λ. The status line converts each
resonance into <b>the road speed that excites it</b>, for a list of bump
spacings you can edit. If washboard puts you on wheel hop at a speed you
actually drive, that is a real setup problem — and now you can see it.</li>
<li><b>Damping is drawn as a band, not a line.</b> A real damper is
asymmetric (rebound is commonly 2–3× bump) and no single linear coefficient
represents that, so the plot shades between the bump and rebound curves: the
car lives somewhere inside. The status line warns if either ratio is
outside racing practice (~0.3–0.5 in bump).</li>
<li><b>Read the limits.</b> This is a <i>linear</i> 2-DOF model — no bump
stops, no progressive springs, no tire lift-off. Treat it as a design-target
and comparison instrument (resonance placement, damping selection, "is this
setup calmer than that one"), not an absolute prediction. It is also a
cross-check: the body mode is computed from an eigenproblem and must agree
with the ride frequency computed in closed form — a regression test enforces
that they do.</li>
<li>📄 <b><a href="__BODE_GUIDE_URL__">Reading the Bode Plot — three-page PDF
guide</a></b> (also under <b>Help ▸ Reading the Bode plot</b>). Covers where
the curves come from, four rules for reading them, <b>what the y-axis should
actually read</b> (peak heights per damping ratio, and how to turn a
dimensionless ratio into tire lift-off or bottoming), and — with sourced
numbers — what "good" looks like for a <b>Baja SAE car</b>, a <b>1:10 scale
RC car</b> and an <b>on-road race car</b>. Two things worth knowing before
you trust the plot: isolation only begins above √2 × the body frequency, and
<i>above</i> that point more damping makes isolation worse, not better.</li>
<li><b>Reading the y-axis in one line.</b> Three of the four channels are
<i>dimensionless ratios per inch of road</i>, so their targets are the same
for any car and are set by the damping ratio: at ζ&nbsp;=&nbsp;0.35 expect a
peak of about <b>2.5</b> on body motion, <b>1.8</b> on tire deflection and
<b>1.9</b> on suspension travel. Body motion always starts at exactly
<b>1.0</b> at DC and tire deflection always ends at exactly <b>1.0</b> at
high frequency — if either asymptote is off, an input is wrong. Body
acceleration is the exception: it is dimensional and scales as f², so
multiply it by the bump height your car actually meets before comparing
anything.</li>
</ul>

<a name="new124"></a>
<h2>What's new in v1.24 (Lotus-style load matrix + envelope)</h2>
<ul>
<li><b>v1.24.1 — the shock bends the arm, and now you can see it.</b> A shock
hanging off the middle of a control arm bends it. That bending is an
<i>internal</i> stress resultant, so by construction it never appears in any
joint reaction — which is exactly why an arm sized as a pure two-force truss
would miss it. <b>File → Export per-part loads + arm bending</b> writes, for
each part, the <b>complete self-equilibrated load set</b> (ΣF = 0, ΣM = 0) to
apply in a component FEA, plus bending / torsion / shear along each arm leg.
On the seeded car the shock puts <b>~10,000 lb·in</b> into the lower arm about
its ball joint. The chassis exports are unchanged and remain correct — the
joint reactions never depended on the bending.</li>
<li><b>v1.24.1 — defect fixed: arm-mounted toe link.</b> When the toe link's
inner ball joint bolts to a control arm (a Setup option the kinematics
already honoured), its reaction on that arm was missing from the arm's
equilibrium, and the mount was reported as a frame pickup. The shock force
was off by up to 3% and individual pickups by far more. Now included, and the
mount is correctly tagged internal. Verified against virtual work for every
shock/toe-link mounting combination.</li>
<li><b>File → Export load-case envelope (friction-circle matrix).</b> A
single impulse along one axis is not the worst case a member sees. This
loads the contact patch with a <b>combined</b> force from the traction
circle — F<sub>x</sub>, F<sub>y</sub> and F<sub>z</sub> together — swept
around every direction (braking, cornering, drive and all combinations),
plus a vertical bump and the two impulse cases. It runs the whole matrix
<b>through the travel range</b> and keeps the <b>worst force per pickup and
worst axial per member</b> — the envelope that actually sizes the hardware.
This is how Lotus Suspension Analysis and standard FSAE practice do it.</li>
<li><b>Auto-derived, self-consistent.</b> The grip (μ by surface) comes from
the tire model and the per-corner vertical load (static + lateral +
longitudinal transfer + aero) comes from the dynamics layer, so the case
already matches the car you're modelling. Pick the loaded corner and
surface; the rest follows.</li>
<li><b>Through-travel.</b> The worst link angle — hence the worst axial
load — is usually at full bump or droop, not at ride height, so the double
wishbone is re-posed and re-solved at each point in its travel. Each member
row lists peak <b>tension AND compression</b> separately (size for yield and
buckling), with the governing case and pose named so you can trace any
peak.</li>
<li><b>Tire moments.</b> The load path now carries the tire's aligning /
overturning couple at the contact patch, not just a point force.</li>
<li><b>Pushrod / bellcrank (opt-in).</b> A rocker load path (pushrod +
damper + pivot bearing reaction) is available for cars that route the wheel
load through a bellcrank instead of a direct coil-over.</li>
</ul>

<a name="new123"></a>
<h2>What's new in v1.23 (impact loads for FEA)</h2>
<ul>
<li><b>File → Export impact loads for FEA (ANSYS/Onshape).</b> Describe an
impact — speed going in, speed coming out, contact time, how much of the
car this corner takes — and the tool runs the impulse chain and pushes the
result through the linkage to give you <b>a force vector at every chassis
pickup</b> plus <b>the axial force in every member</b> (tension or
compression), as a CSV you can load straight into ANSYS or Onshape
Simulation.</li>
<li><b>The physics.</b> Impulse: a hit that changes the car's speed by Δv
in a contact time Δt needs an average force <b>F = m·Δv/Δt</b>. Real
impacts aren't rectangular pulses, so the <b>pulse shape</b> scales that
average up to the design peak (half-sine ×1.57, triangular ×2.00). Then
static equilibrium of the corner — the upright's six equations plus one
moment equation per arm about its own bushing axis — gives the load path.
Presets cover a head-on rock strike, a lateral kerb strike, a 3&nbsp;ft
landing and a square-edge bump; a <b>drop height</b> box fills the speed
from v&nbsp;=&nbsp;√(2gh).</li>
<li><b>Every suspension type is covered.</b> The double wishbone is a fork
(two arms into one upright); the multilink is a square 6×6; and the
trailing arm, H-arm, C-hub and loaded halfshaft are solved as serial chains
of rigid bodies, outboard to inboard — for the C-hub that is steering block
→ C-hub → lower arm. Rows are tagged <b>chassis</b> (apply these to a frame
FEA) or <b>internal</b> (ball-joint / hinge / kingpin loads for sizing the
joint and upright — they cancel inside the corner).</li>
<li><b>Verified three ways, on all six types:</b> the chassis mount loads
sum back to the wheel load in force AND moment (machine precision), and the
shock force independently matches wheel&nbsp;load × motion&nbsp;ratio from
the kinematic sweep — exactly, for every type.</li>
<li><b>Read the limits before trusting a stress number:</b> this is
<i>quasi-static</i> — the peak force is applied as a static load, with
rigid links and no bushing compliance or structural dynamics. That's the
standard way to size links by hand, and it's why the pulse shape matters.
The axial split between the two bushings of one arm is statically
indeterminate; the CSV notes the total axial load so you can check a
bracket against all of it rather than half.</li>
</ul>

<a name="new1221"></a>
<h2>What's new in v1.22.1 (dynamics audit + tire data)</h2>
<ul>
<li><b>Dynamics numbers audited against outside sources; three real fixes.</b>
Every dynamics output was re-derived against SAE J670 / Milliken / Gillespie
/ OptimumG. Three genuine errors were corrected, so a few numbers now differ
from the old spreadsheet: (1) <b>lateral load transfer</b> used the total
weight in the roll-moment term (the roll gradient already used the sprung
weight) and dropped the unsprung-at-wheel reaction — it overstated transfer
~12%; now the full Milliken 18.4 form. (2) <b>Bounce / pitch frequencies</b>
used the wheel rate, which put the bounce frequency <i>above</i> both corner
ride frequencies (impossible); now the ride rate, like the ride frequencies.
(3) <b>Banking</b> sign is now SAE-standard (positive bank = into the turn =
less tyre demand). Everything else audited correct or a documented
convention.</li>
<li><b>Estimated tire data for the SUN-F 23x7-10.</b> The tire module
(<code>tire.py</code>) now carries a full estimated parameter set — vertical
rate, peak lateral grip by surface (0.4 mud … 0.8 pavement, ~0.7 dry
hardpack), cornering stiffness, peak-slip angle, camber guidance, rolling
resistance — with EVERY number tagged by confidence (measured / derived /
informed-estimate / rough-guess) and the reasoning used. There is still no
measured F&M data, so treat outputs as qualitative. Key design takeaway: a
knobby tire wants <b>near-zero static camber</b> (large camber lifts the
inside lugs and shrinks the patch) and peaks at a high slip angle (~10-15°),
and its cornering stiffness is much lower than the dynamics sheet's default
— worth revisiting that input.</li>
</ul>

<a name="new122"></a>
<h2>What's new in v1.22 (inner-joint type + directional plunge budget)</h2>
<ul>
<li><b>Pick the inner (diff-side) joint type.</b> The Halfshafts panel has
a joint-type dropdown: <b>Plunging CV</b> (tripod/AAR/VL — plunges inside
the joint, ~25&deg;), <b>Rzeppa</b> (fixed CV — ~47&deg; but does NOT
plunge), and <b>Double-cardan U-joint</b> (~45&deg;, plunge taken by an
<b>extending slip-spline shaft</b>). Choosing a type fills a sensible
default angle limit (still editable) and enables/disables the plunge
budget. A fixed Rzeppa inner can't plunge, so if the geometry actually
needs shaft-length change the tool warns you to use a plunging joint or a
slip shaft.</li>
<li><b>Plunge budget is now directional.</b> Instead of a single &plusmn;
number, you set a <b>total plunge stroke</b> plus a <b>plunge-in %</b> —
how much of that stroke is available for plunge-IN (compression) from the
ride-height shaft length; the rest is pull-OUT (extension). Because ride
height rarely sits centred in the joint's travel on a bump-biased car,
this lets you bias the budget the way the shaft actually moves (same idea
as the shock's bump/droop split). Each direction is checked against its
own allowance through travel. Old projects migrate automatically (the old
&plusmn;25&nbsp;mm becomes a 50&nbsp;mm stroke, 50/50 split).</li>
</ul>

<a name="new1212"></a>
<h2>What's new in v1.21.2 (separate inner/outer CV angle limits)</h2>
<ul>
<li><b>The two CV joints now have their own angle limits.</b> The inner
(plunging &mdash; tripod/AAR/VL) joint tolerates much less articulation
(~22&ndash;33&deg;) than the outer fixed (Rzeppa) joint (~45&ndash;50&deg;),
and on a DRIVEN FRONT the outer joint also swallows the steering angle.
A single shared cap was therefore too tight on the outer and too loose on
the inner. The Halfshafts panel now has two boxes &mdash; "Max CV angle —
inner (plunge)" (default 25&deg;) and "Max CV angle — outer (fixed)"
(default 45&deg;) &mdash; and each joint's peak angle is checked against
its own limit, with the warning naming the joint. Old projects with a
single limit load unchanged (that value migrates onto both). The graph
still shows the absolute worst-joint angle.</li>
</ul>

<a name="new1211"></a>
<h2>What's new in v1.21.1 (roll-centre / anti accuracy fix)</h2>
<ul>
<li><b>More accurate roll centre and anti-geometry once the arms are
inclined fore/aft (kickup or anti).</b> The roll centre is (correctly)
built in the FRONT VIEW &mdash; the transverse vertical plane, the SAE
J670 definition &mdash; but the double-wishbone construction used to
collapse the 3-D arms into that plane by slicing both control arms at the
single wheel-centre plane. That is only exact for flat (zero-kickup)
arms; with kickup it mislocated the front-view instant centre and read
the roll centre a bit low (and made it look almost flat vs kickup). Each
arm is now sliced at its OWN ball-joint station, which matches rigid-body
kinematics exactly. On a 10&deg;-kickup car the front RC reads a few mm
higher and now correctly RISES with kickup (~1&nbsp;mm/deg); at 20&deg;
the correction is ~25&nbsp;mm. The same fix is applied to the side-view
instant centre (anti-dive / anti-squat) &mdash; anti is UNCHANGED for the
one-sketch (parallel-arm) seeds and corrected only when you deliberately
skew the arms for anti. No change at all at zero kickup.</li>
</ul>

<a name="new121"></a>
<h2>What's new in v1.21 (H-arm rear)</h2>
<ul>
<li><b>H-arm (rear)</b> (Suspension type dropdown) &mdash; a simple,
robust driven rear end. A <b>rigid lower H-arm</b> (two inboard bushings
on a fore-aft axis) grabs the upright at <b>two outboard points</b>, so
the lower arm alone holds toe and locates the bottom of the upright &mdash;
no separate toe link. A single <b>upper camber link</b> sets camber and
closes the geometry. Driven by default: the CV halfshaft is modelled as
the usual angle/plunge check (disable it in the Halfshafts panel for a
dead axle). Toe stays essentially fixed through travel (the wide grab
base is what holds it); to add gentle passive rear steer, skew the
<b>grab line</b> in the Kinematic tweaks panel. Shock mounts on the lower
H-arm. Closed form (same hinge-carrier core as the C-hub / loaded
halfshaft); roll centre via the numeric instant-centre method.</li>
</ul>

<a name="new120"></a>
<h2>What's new in v1.20 (two new suspension types)</h2>
<ul>
<li><b>C-hub front</b> (Suspension type dropdown) &mdash; the 1/10-scale
RC buggy front end (think Team Associated B7), scaled up: a lower arm
carries a <b>hinge pin</b> at its outer end, the <b>C-hub</b> (caster
block) rotates on that pin, a single <b>camber link</b> from a chassis
ball (near the tower &mdash; it's just a point, put it where your chassis
allows) closes the loop, and the steering block pivots on a <b>physical
kingpin fixed in the C-hub</b>. Caster / KPI / axle offset are therefore
hardware properties of the blocks &mdash; tune all three directly in the
Kinematic tweaks panel (like swapping caster blocks and KPI inserts).
The seed aims the kingpin at your desired caster & scrub, aims the
camber link at a sensible inboard IC, and secant-tunes the tie-rod inner
height to near-zero bump steer at ride.</li>
<li><b>Loaded halfshaft (rear)</b> &mdash; the halfshaft as a
<b>structural lateral link</b>: an upper arm (shock on it) carries a
bushing pin at its outer end, the knuckle rotates on that pin, and the
<b>shaft itself</b> (diff flange &rarr; outer CV, fixed length) locates
the knuckle &mdash; no toe link needed (the pin fixes toe; skew it in
the tweaks panel for gentle passive steer). The diff flange
(<i>hs_inner</i>) is a real hardpoint: edit it in the table, drag it, or
type it in the Halfshafts panel &mdash; all three move the suspension.
<b>Plunge reads exactly 0 by construction</b> (a loaded shaft cannot
plunge) &mdash; if it doesn't, something is wrong; CV angles remain the
numbers to watch.</li>
<li>Both types are closed-form (same math family: arm &rarr; carrier on
a hinge pin &rarr; one closing link), fully live in the table / 3D /
plots / sweeps / export / optimizer, and save into .MICK like everything
else. Roll centres use the numeric (Adams-style) instant-centre method
&mdash; the classic two-arm-plane construction doesn't apply to a
carrier on a hinge pin.</li>
</ul>

<a name="new119"></a>
<h2>What's new in v1.19 (one origin for the whole car)</h2>
<ul>
<li><b>The rear axle now reads &amp; enters coordinates in the same
frame as the front.</b> Previously each axle's hardpoint table and
halfshaft entry used that axle's own local origin, so a rear point had no
shared fore-aft datum &mdash; hard to type in, and it didn't match
Onshape. Now both axles reference ONE origin (the front / firewall
datum): the rear is shown a wheelbase behind the front, exactly like the
CSV / Onshape export and the 3D scene already did. Type a rear halfshaft
or hardpoint straight from your CAD numbers. (Projects still store
axle-local mm internally, so old files load unchanged; only the
display/entry frame changed. The offset tracks the wheelbase.)</li>
</ul>

<a name="new118"></a>
<h2>What's new in v1.18 (arm-mounted toe link — physics corrected)</h2>
<ul>
<li><b>Mounting the toe link on a control arm does NOT fix toe.</b>
Earlier notes wrongly implied the arm-mounted link tracked the camber
curve and couldn't be tuned. The truth: bolting the inner ball joint to
the same arm as the lower/upper ball joint forms a near-rigid
BJ&ndash;inner&ndash;steering-arm triangle, so the steering arm follows
that arm and the toe changes a lot through travel &mdash; <b>large bump
steer</b>, exactly what you'd expect from taking a chassis-designed link
and bolting it to the arm. The solver already modelled this correctly;
only the wording (and the seed's outboard placement) needed fixing.</li>
<li><b>Lower AND upper arm options.</b> Two Setup checkboxes (mutually
exclusive; both off = chassis). The inner ball joint's centre rides the
chosen arm; the tie rod still pivots freely at both joints.</li>
<li><b>The seed minimizes the (unavoidable) bump steer</b> by seating the
OUTBOARD link low on the kingpin for a lower-arm mount, high for an upper
&mdash; keeping the tie-rod IC near the arm's IC. That, plus the
steering-arm (outer) tweak, is the tuning lever; the inner's position
along the arm barely changes bump steer. The seed reports the real value
honestly.</li>
</ul>

<a name="new117"></a>
<h2>What's new in v1.17 (measurement audit + view toggles)</h2>
<ul>
<li><b>Every user-facing number was independently verified.</b> A
13-cluster audit re-derived each measurement by alternative means — a
from-scratch constraint solver, hand-built rotation matrices, symbolic
line intersections, first-principles Milliken formulas — and confirmed
the solver, wheel angles, scrub/trail, roll centres, steering/Ackermann,
CV geometry, sweeps and the whole dynamics panel are correct to machine
precision. Four small issues were found and fixed:</li>
<li><b>Camber/toe now read back exactly.</b> The spindle is built so both
the front-view camber and top-view toe reproduce your input for any
combination (the old form coupled a tiny error into camber at large
toe).</li>
<li><b>Anti-dive/squat/lift are continuous at the design state.</b> At
the parallel-arm geometry the shipped seeds sit at, the anti % now equals
its geometric limit and no longer jumps ~2-4 points when the arms are
skewed a hair (a fallback-construction mismatch); your rear anti-squat
readings shift a couple of points to the correct value.</li>
<li><b>Half-track change &amp; wheel recession</b> are referenced to true
static (travel 0), not the nearest sweep sample — removes a ~1 mm offset
on asymmetric travel ranges. <b>Motion ratio &amp; bump steer</b> are now
second-order accurate at the full-bump/droop endpoints too.</li>
<li><b>Separate Roll-centre / IC view toggles</b> (v1.16.2): hide the
roll-centre dots and the instant-centre markers independently to declutter
the 3D view for new users.</li>
</ul>

<a name="new116"></a>
<h2>What's new in v1.16 (shock hardware truth + locks are absolute)</h2>
<ul>
<li><b>Fixed shock hardware spec.</b> The shock's compressed/extended
lengths now live with the axle in the <b>Halfshafts &amp; shocks</b>
panel — datasheet numbers you know before you start, persisted in the
.MICK and never lost to a reseed (a spec overrides the seed form's
fields). On old files the spec is migrated automatically.</li>
<li><b>The travel range IS the shock's range (v1.16.1).</b> With a spec
set, the range follows the hardware stroke AUTOMATICALLY on every edit,
load and reseed — expanding and shrinking so the shock always sweeps
max→min exactly (or to a linkage lock, whichever comes first; the
status says which). The range spins become displays. One real front
showed travel needing 128% of the stroke (impossible); the rear capped
the shock at 23.3 in of its 24.68 in extension — both now land on the
hardware lengths to a tenth of a mm with no button pressing. CV angle
and plunge over the range stay pure WARNINGS (Halfshafts panel +
checklist) — they never cap travel. Two new Readouts rows show the
stroke split at ride and the % of stroke used.</li>
<li><b>Cleaner view for new users (v1.16.2).</b> The old combined
"RC/IC/axis" toggle is now two independent checkboxes in the view
controls: <b>Roll centres</b> (the RC dots + the roll axis line) and
<b>IC markers</b> (the front/side-view instant centres + their
construction rays) — hide either without losing the other.</li>
<li><b>Locks are absolute now.</b> Nothing moves a pinned point: the
ride-height and corner fore/aft tweaks and the vehicle ride-height
knob refuse with an explanation (vehicle RH refuses atomically — no
half-moved car), and the optimizer skips locked points from its free
set. Locked spheres draw <b>amber</b> in 3D so pinned points are
unmistakable.</li>
</ul>

<a name="new115"></a>
<h2>What's new in v1.15 (kickup as a tweak + reseed that honors locks)</h2>
<ul>
<li><b>v1.15.2 — vehicle ride height in one knob.</b> The Vehicle panel
gains <b>"Vehicle ride height (both axles)"</b>: type a number, hit Set,
and BOTH axles rigidly translate to that measured ride height (each
using its own frame-tube datum) — the whole car's stance without a
reseed, and the one-click fix for a red F/R stance row. No kinematics
change; each axle's move lands in its own undo. The spin tracks the
front axle's live measured value.</li>
<li><b>v1.15.1 — ride height &amp; stance are one source of truth.</b>
The Readouts ride height is now MEASURED from the displayed geometry
(ground → frame datum, identical to the tweak row) instead of echoing
the Setup form's stored number, which went stale whenever a tweak or
drag moved the corner in z — that's why the 3D wheels could sit at
different heights while the readouts claimed they matched. The Vehicle
panel gains an <b>F/R stance</b> row that goes red when the two axles'
ground planes differ (one car, one ground — equalize the ride-height
tweaks). Cross-axle numbers (roll-axis angle, height at CG, moment arm,
dynamics link) now reference both roll centres to the FRONT axle's
ground plane, matching the 3D axis exactly even when the stance is off.
And the <b>Ride height / Reset pose</b> buttons reset BOTH axles now,
so a parked slider can't leave one end posed and looking misaligned.</li>
</ul>
<ul>
<li><b>Sketch kickup is now a live tweak.</b> The Kinematic-tweaks panel
re-angles the four inboard bushing axes IN PLACE (tube centres, spreads
and the sketch yaw all stay put) and re-planarizes the kingpin so caster
follows, exactly like a fresh seed — with nothing else moving. This is
the knob for "how does kickup move my roll centre" studies; no reseed
needed. On a real front corner, kickup raises the front RC ~3.4 mm per degree
and strengthens camber gain with it.</li>
<li><b>Reseeding honors locked hardpoints.</b> Pin your welded tabs /
fixed mounts (double-click in the Hardpoints table), then Generate: the
new seed is slid onto the pinned points (X/Z, kinematics preserved) and
they are kept EXACTLY — the reseed builds around them instead of
relocating them through the placement heuristics.</li>
<li><b>Landing warning.</b> If a reseed still lands far from the previous
design, the status bar says how far and reminds you to Measure setup
first (and lock fixed tabs) to reseed in place.</li>
</ul>

<a name="new114"></a>
<h2>What's new in v1.14 (one source of truth + reseed without loss)</h2>
<ul>
<li><b>Kingpin-off-sketch now explains itself.</b> The sketch-fit line
shows the reconciliation — measured caster vs measured kickup (and yaw).
Important: <i>off-plane = caster − kickup only on an UN-YAWED sketch.</i>
With sketch yaw (an angled rear), the kingpin's KPI lean also tilts it
relative to the plane, so 10° caster over a 5° kickup can legitimately
read ~2° off-plane. Every number comes from the same measured 3D
geometry — the readout now shows the pieces so you can check it.</li>
<li><b>Wheelbase readout is live</b>: static wheelbase plus each axle's
wheel recession at its displayed pose (wheel centres move fore/aft
through travel), not the seed's static number.</li>
<li><b>Corner fore/aft tweak</b>: a new Kinematic-tweaks row rigidly
slides the WHOLE corner (sketch plane, arms, knuckle, shock) along the
chassis-forward axis — retune the seed's placement without reseeding.
Measured at the wheel centre's X station.</li>
<li><b>Reseed without losing your design</b>: <b>Setup → Measure setup
from current design</b> fills the form from the ACTIVE axle's live
geometry (track, scrub, caster, kingpin span, arm gap, motion ratio,
sketch kickup/yaw, placement…). Flip the one seed-only option you need
and Generate — the reseed lands on the measured design, not the
defaults. Load a candidate first to measure that instead. (One-sketch
mode still planarizes caster to the kickup on reseed; use
independent-caster to keep a caster that differs.)</li>
<li><b>⊥-sketch column</b>: the Hardpoints table's fourth column shows
each point's offset along the sketch normal. Edit it to slide a point
perpendicular to the 2D sketch — packaging moves that mostly preserve
the in-sketch kinematics (bushings slide along their own axes, so those
are exactly kinematics-neutral).</li>
</ul>

<a name="new113"></a>
<h2>What's new in v1.13 (flexibility + cleanup)</h2>
<ul>
<li><b>v1.13.1 — steering link on a control arm</b>: Setup checkboxes
("Steering link on LOWER / UPPER arm") for a rear double wishbone whose
toe link's inner ball joint mounts on a control arm instead of the
chassis. Its centre then rides that arm through travel (drawn attached in
3D, saved in the .MICK). <b>This does NOT fix toe</b> — see the v1.18
note; the framing here was corrected.</li>
<li><b>v1.13.1 — drag fix</b>: locking a hardpoint no longer scrambles
which sphere drags which point (locking the inboard arm points used to
make the steering-link spheres grab the wrong hardpoints).</li>
<li><b>Retarget scrub &amp; caster without reseeding</b> — the Kinematic
tweaks panel has new <b>Scrub radius</b> and <b>Caster angle</b> rows.
They lean / tilt the kingpin (moving the UBJ) to hit the number you type,
keeping track and static camber/toe — so you no longer regenerate a whole
seed to change scrub or caster after hand-placing the arms. (Caster set
this way holds until you Re-square, which pulls the kingpin back into the
sketch plane; use independent-caster mode to keep it.)</li>
<li><b>Lock hardpoints</b> — double-click a point's name in the Hardpoints
table to pin it (🔒). A locked point won't drag and the scrub/caster
tweaks refuse to move it, so fixed halfshaft mounts and chassis points you
like stay exactly put. Locks save in the .MICK.</li>
<li><b>Wheelbase actually takes effect</b> — the seed's wheelbase now
drives the live Vehicle-panel value (rear-axle station, CG split, export)
instead of sitting inert; keep tuning it in the Vehicle panel.</li>
<li><b>Live readouts fixed in Roll mode</b> — track width, wheelbase and
ride height no longer blank out when you roll; every motion mode feeds the
Readouts panel through one path now.</li>
<li><b>Cleanup</b> — removed dead Onshape handlers, corrected stale menu
labels (New, Dark theme) and reconciled this guide to the single
Build/update CAD flow. A placeholder tire seam (tire.py) is in for future
grip-driven targets — no real SUNF data exists yet, so it is clearly
marked as a guess.</li>
</ul>

<a name="start"></a>
<h2>Getting started — your first design, step by step</h2>
<ol>
<li><b>Load your chassis</b> — Frame panel → load the STL/3MF export of
your frame. It auto-scales and auto-rotates onto the chassis
convention; with the default seed offsets your suspension will land ON
the frame with no manual transform. (If you're just exploring, skip
this — everything works without a frame.)</li>
<li><b>Set the Setup values</b> — track width, wheelbase, ride height,
tire size, your shock's lengths, motion ratio goal. Tick
<b>"Driven axle"</b> if this axle runs halfshafts (do this BEFORE
seeding — the CV axle is placed at seed time). If this axle's frame
tubes run at an angle (like an angled rear), set <b>Sketch plane
yaw</b> to that angle.</li>
<li><b>Generate seed hardpoints</b> — a complete, buildable corner
appears, already matched to your numbers and mounted at the Onshape
frame origin. Repeat per axle (the Editing dropdown switches
front/rear).</li>
<li><b>Drag mounts where your frame needs them</b> — click a sphere in
the 3D view and drag; tap X / Y / Z while dragging to lock an axis;
Esc cancels. Or type exact numbers in the Hardpoints table. Suspension
mounts usually sit on welded tabs OFF the tube centreline — that's
fine; the optional "snap to frame" toggle exists when you do want the
surface.</li>
<li><b>Watch the Design Check</b> — every edit re-grades articulation,
one-sketch buildability, bump steer, roll-centre band, CV limits, and
turning radius. Hover a ✗ for the number behind it. Green board =
buildable design.</li>
<li><b>Tune</b> — the Tweaks panel speaks designer dimensions (arm
lengths on the sketch, kingpin length, inboard axis gap, steering arm,
tie-rod rises, hub position) instead of raw coordinates. The Optimize
panel refines chosen goals (camber gain, bump steer, RC height…) by
moving only the hardpoints you free.</li>
<li><b>Snapshot Candidates</b> — every design worth keeping goes in the
Candidates dock (rename them; the metric table compares them
side-by-side). Save the project as a .MICK file; autosave runs in the
background.</li>
<li><b>Export</b> — CSV reports, hardpoint tables in your CAD
convention, or push the whole design + a Variable Studio straight into
your Onshape document.</li>
</ol>

<a name="design"></a>
<h2>Designing a Baja suspension — the numbers that matter</h2>
<p><i>A crash course in what to aim for and which knob moves each
number. The Design Check encodes most of this; here's the why.</i></p>
<ul>
<li><b>Travel</b> — Baja terrain rewards travel biased toward bump
(e.g. +10&nbsp;in bump / −4&nbsp;in droop). Set it in the travel spins;
if an edit can't reach it, the range auto-clamps and the checklist
shows the shortfall.</li>
<li><b>Camber curve</b> — you want mild negative camber gain in bump
(~−0.03 to −0.05 deg/mm) so the outside tire stays flat in roll.
Knobs: UCA length on the sketch (shorter upper arm = more gain),
kingpin length, and the inboard UC–LC axis gap. <b>Keep the inboard
axis gap well under the kingpin length</b> — as it approaches it, the
front-view instant centre crosses over and the camber curve reverses
direction.</li>
<li><b>Roll centre</b> — target a static height around 0–12&nbsp;in
above ground (band editable in the checklist) and watch that it
doesn't migrate wildly through travel. Raising inboard pivots raises
the RC.</li>
<li><b>Bump steer</b> — under ~0.01 deg/mm at ride. Tune with the
tie-rod rise knobs in Tweaks: the tie rod should point at the
instant centre like the arms do.</li>
<li><b>Caster / KPI / scrub</b> — caster (4°-ish) gives self-centring;
KPI follows from your scrub-radius target (~1&nbsp;in). Both are set
in the seed and visible live in the readouts. Caster lives in the
ball joints' fore-aft offsets — the sketch's job is arms and camber,
so Re-square never touches it.</li>
<li><b>One 2D sketch</b> — the arms must be buildable from ONE sketch
with bushing tubes normal to it: both axes parallel AND at the axle's
design yaw. Hand edits drift this; <b>Re-square arms</b> restores it
with the smallest possible move.</li>
<li><b>Halfshafts</b> — place the inner CV level with the outer at
MID-travel (the seed does this) to halve plunge over a bump-biased
range. Keep worst CV angle under the joint rating (~35°) and plunge
inside its axial travel; the checklist and plots track both.</li>
<li><b>Steering</b> — set your rack travel and turning-radius goal in
the checklist thresholds; watch the Ackermann graph (100% = ideal
geometry for low-speed cones, 0% = parallel). The steering-arm tweak
trades effort against max road-wheel angle.</li>
<li><b>Dynamics</b> — the Dynamics panel carries the full Milliken
sheet (ride/roll rates, load transfer) once you have masses and
spring rates; kinematic mode hides what you don't need early.</li>
</ul>

<a name="new19"></a>
<h2>What's new in v1.9</h2>
<ul>
<li><b>MICKSUS</b> — the tool has a name, a splash screen, and boots in
dark mode with a BLANK workspace: load your frame, then seed (no more
surprise default car).</li>
<li><b>Twisted sketch plane</b> — set <b>Sketch plane yaw</b> in Setup
for axles whose frame tubes run at an angle. Seeds generate at that
angle, squareness is judged against it, and Re-square rotates existing
geometry onto it (your wheel stays pointing straight; expect the
wheelbase to breathe a few mm through travel — that's real).</li>
<li><b>Sketch-dimension tweaks</b> — kingpin length (UBJ slides),
direct inboard UC–LC axis gap, and each arm's length as dimensioned on
the sketch (BJ slides in-plane, caster kept). Move-and-flag: the tweak
always applies; the checklist calls out anything it breaks.</li>
<li><b>Static toe by tie rod</b> (v1.10) — the "Static toe (threads the
tie rod)" tweak rotates the whole knuckle about the kingpin until the
wheel reads your target toe at ride height, just like threading the tie
rod on the car: the wheel, hub and tie-rod knuckle point swing
together, the link length changes to suit, and a leaned kingpin nudges
camber slightly (the stored alignment tracks it). Use this instead of
typing static toe when you want the GEOMETRY to match, not just the
spindle aim.</li>
<li><b>Passive-steer optimizer goal</b> (v1.10) — a new
"Toe vs travel" goal lets you drive toe to CHANGE through travel at a
chosen deg/mm rate instead of staying flat: e.g. a fixed-tie-rod rear
that toes OUT in bump and IN in droop (target negative in the
toe-in-positive convention) for passive rear steering.</li>
<li><b>Panel font size</b> — Panels menu → Panel font size, so the
whole hardpoint table fits on screen. Persists.</li>
<li>This guide grew the <a href="#start">Getting started</a> and
<a href="#design">engineering</a> sections.</li>
</ul>

<h2>Panels are dockable (v1.3)</h2>
<p>Every design panel (Setup, Hardpoints, Tweaks, Optimize, Vehicle,
Dynamics, Frame, Readouts) lives in its own dock: <b>drag the title bar</b>
to move it, drop it on another panel to tab them together, drag it out of
the window to float it (second monitor!), or close it. Reopen anything
from the <b>Panels</b> menu.</p>

<h2>The workflow (the panels, in design order)</h2>
<ol>
<li><b>Setup variables</b> — the numbers you know before any geometry
exists (track, wheelbase, tire, shock lengths, motion-ratio goal…).
<b>Generate seed hardpoints</b> turns them into a valid starting
geometry using rules of thumb, with the shock mount and tie rod
auto-tuned to hit your motion ratio and near-zero bump steer.</li>
<li><b>Hardpoints table</b> — raw X/Y/Z editing. Every edit re-solves;
edits that make the linkage impossible are rejected and reverted (watch
the status bar). Coordinates are the LEFT corner: +X forward, +Y left,
+Z up, origin on the ground at that axle's centreline.</li>
<li><b>Kinematic tweaks</b> — the dimensions designers actually think
in (shock mount along the arm, tie-rod rises, pivot skew…). Same live
re-solve, different handles.</li>
<li><b>Optimization goals</b> — pick targets (camber gain, bump steer→0,
roll-centre height…), pick which hardpoints may move, hit Run. The
before/after shows as dashed curves + a ghost linkage.</li>
<li><b>Vehicle</b> — wheelbase, CG, brake split → roll axis, roll moment
arm, weight split, anti-dive/anti-squat percentages. The CG shows as a
yellow sphere in the 3D view (toggle: the <b>CG</b> checkbox) with its
coordinates listed in the panel.</li>
<li><b>Dynamics</b> — the Milliken &amp; Milliken (RCVD) spreadsheet,
live. In the default <i>kinematics mode</i> you only see the numbers a
geometry designer needs (roll gradient, weight transfer, corner loads,
ride frequencies); tick <b>Full dynamics mode</b> for everything —
springs, dampers, ARBs, tires, brakes, aero, understeer. With
<b>Link kinematics</b> on (default), the roll-centre heights, CG,
wheelbase, motion ratios and the EXACT anti-percentages stream in from
your live model (fields marked 🔗 lock while linked). Units here are the
reference units — lb, in, lb/in, lb-ft/deg — regardless of the display
toggle. Since v1.28 the motion ratio uses the SAME convention on both
sides — shock/wheel, below 1 — so the live link passes it straight
through with no inversion.</li>
<li><b>Frame backdrop</b> — load your chassis STL/3MF and line it up to
eyeball clearances.</li>
</ol>

<h2>What's new in v1.8</h2>
<ul>
<li><b>The "blue dot" orbit bug is gone</b> — the far-away blue dot was
the side-view instant-centre marker, and it was dragging the camera's
idea of the scene with it. Orbit recentering and Fit view now measure
the CAR only; instant-centre markers farther than 4&nbsp;m are hidden
(the direction ray stays, clamped).</li>
<li><b>Outer CV rides the tire's centre axis</b> — it sits where the
kingpin axis meets the wheel-spin axis (the physical CV location), with
a thin stub drawn from the CV to the wheel centre so you can see it.
</li>
<li><b>Squareness check</b> — the one-sketch row now also flags bushing
axes <i>yawed out of the side view</i> (the "you can see the other side
of the kingpin" defect). <b>Re-square arms</b> removes the yaw too;
ball joints (your caster) are never touched.</li>
<li><b>Steering arm tweak</b> — kingpin axis → tie-rod outer, measured
along the sketch normal. Longer arm = lighter steering but less road-
wheel angle for the same rack travel. Static toe is unaffected.</li>
<li><b>Turning radius &amp; Ackermann in the Design Check</b> — set the
rack's ± travel and your turning-radius goal in the thresholds; the
checklist grades the outside-front-wheel radius at full lock at ride
height and draws an Ackermann-% graph over the rack sweep.</li>
</ul>

<h2>New in v1.12</h2>
<ul>
<li><b>Hub offset tweak (outer CV &rarr; tire centre)</b> — the
hub/knuckle stickout: how far the wheel centre stands off the outer CV
joint. Sliding it moves the wheel along its spin axis, so scrub radius
and half-track change while camber, caster and KPI stay put. This is the
number to set for hub/bearing/CV-cup packaging. (The Setup form's
separate "Hub offset (WC&rarr;BJ plane)" still seeds the lateral
ball-joint position.)</li>
<li><b>Shock length @ ride tweak</b> — set the installed (eye-to-eye)
length by sliding the arm mount; the line under it shows the
<b>bump/droop stroke split</b> so you can dial a target travel budget.</li>
<li><b>Ride height tweak</b> — a rigid vertical placement (no kinematics
change), driven by the new <b>Frame-tube offset</b> Setup variable
(default 5&frasl;8&nbsp;in) that puts the ride-height datum above the CAD
origin. The Setup X/Z "seed placement" offsets are the same rigid-shift
knobs as before, just clearly labelled.</li>
<li><b>Hover a graph</b> to read the exact metric value at the cursor's
travel instead of guessing between grid lines.</li>
<li><b>Free (L/R) travel mode</b> — the motion dropdown gains a mode that
gives the <b>right wheel its own slider</b>. Hold one wheel at ride
height and bump the other to read the single-wheel-bump roll centre; the
roll axis updates from the asymmetric pose.</li>
<li><b>Export menu trimmed</b> to the essentials — the older "Push
hardpoints" and "Sync design" Onshape items are gone; <b>Build/update
CAD</b> supersedes them.</li>
<li><b>Outer CV on the tire centreline</b> (v1.12.1) — the CV output line
is now coincident with the wheel-spin axis, not angled to it (see
Halfshafts below).</li>
<li><b>Wheelbase &amp; track width</b> added to the Readouts fly-out next
to ride height (track width is measured live from the geometry, so it
breathes with the wheels).</li>
<li><b>Knuckle-consistency convention</b> (v1.12.3, always on) — the tire
spin axis is kept in the plane of the two ball joints and the wheel
centre, and the steering-arm plane sits at the <b>Steering-arm plane
angle</b> you set (default 90&deg;) to it. See below.</li>
</ul>

<h2>Knuckle convention (v1.12.3)</h2>
<p>Real knuckles are built so the CV output line runs straight down the
tire centreline and the steering arm sticks out at a set angle to the
wheel. MICKSUS now enforces that automatically: looking down the kingpin,
the two ball joints and the wheel centre define the <b>tire plane</b>, and
the tire's spin axis is kept <b>in</b> that plane (so the CV joint and its
3D stub are coincident with the tire centreline, never cocked to it). The
<b>steering-arm plane</b> (ball joints + tie-rod outer) is held at the
<b>Steering-arm plane angle</b> from the tire plane — set it in Setup
(default 90&deg;).</p>
<p>It's applied by sliding the wheel centre the smallest <b>fore/aft</b>
amount onto the tire plane, so camber, toe, track and ride height are
untouched (scrub barely moves), and by swinging the tie-rod outer to the
angle with the rack end fixed, so static toe is unchanged (only steering
ratio / Ackermann). It runs on every seed and edit and is idempotent, so
opening an older design just snaps it consistent.</p>

<h2>Build CAD in Onshape (v1.11)</h2>
<p><b>File &rarr; Build/update CAD in Onshape</b> does the full push:
it writes a <b>Variable Studio</b> of every hardpoint, adds a custom
FeatureScript feature that turns those variables into <b>3D points</b>
and the <b>front/rear 2D sketch planes</b>, and drops in a
<b>design-notes PDF</b> (readouts + curves). Enter your document URL,
free API keys (dev-portal.onshape.com/keys), and the Part Studio name.
Beyond points it also builds, per axle: the <b>lower/upper A-arm
planes</b>, the <b>2D design sketch plane</b>, the <b>halfshaft CV
points</b>, and a full <b>construction-line skeleton</b> (bushing axes,
kingpin axis, both A-arm legs, tie rod, shock, halfshaft) as named
reference lines to model tubes against. The feature is
<b>self-contained</b> &mdash; it embeds the coordinates, so it builds
even without inserting the Variable Studio, while still letting a live
#variable override move geometry.
The best part: it is <b>idempotent</b> &mdash; MICKSUS remembers the
tabs it made (stored with your project), so pushing again after a
redesign <b>updates those same tabs (and the feature itself) in place
instead of duplicating them</b>, so your downstream references survive.
Lose the project? It still finds the tabs by name and the feature by
type.</p>

<h2>Toe by tie rod &amp; passive steer (v1.10)</h2>
<p>The Tweaks panel's <b>"Static toe (threads the tie rod)"</b> sets toe
the mechanic's way: it rotates the knuckle about the kingpin until the
wheel reads your target at ride height, so the geometry (not just the
stored number) matches. For a fixed-tie-rod rear, the optimizer's new
<b>"Toe vs travel"</b> goal targets a toe RATE through travel &mdash;
set it negative to toe out in bump and in during droop (passive rear
steer).</p>

<h2>Halfshafts v2 (v1.7)</h2>
<p>The outer CV joint sits <b>on the tire centreline</b> (v1.12.1) — the
wheel-spin axis through the wheel centre, at the point nearest the
kingpin — so the CV output line is <b>coincident with the tire
centreline</b>, not cocked at an angle to the wheel (an angled CV is
buildable but out of scope). Both CV joints appear as extra <b>rows in
the hardpoint table</b> whenever the axle has a halfshaft. The inner CV
is a free chassis point (type the gearbox-flange coordinates the
powertrain team fixes) and is also <b>draggable</b> in the 3D view.
Editing the outer row slides the hub along the kingpin — the tire height
moves with it — and the same knob lives in the Tweaks panel as
"Hub/outer-CV along kingpin". Enabling a halfshaft whose inner point is
obviously off-axle auto-places it at the station, level at mid-travel
(the minimum-plunge placement).</p>

<h2>Camera orbit (v1.7)</h2>
<p>The rotation pivot re-anchors onto the model after every
orbit/pan/zoom, so the camera can no longer end up circling a point off
in space. If it ever looks wrong anyway, <b>Recenter orbit</b> (next to
Fit view) fixes it without moving the picture.</p>

<h2>Re-square arms (v1.7, extended v1.11)</h2>
<p>The Optimize panel's <b>"Re-square arms"</b> button repairs
one-sketch bushing-axis drift on demand, without running any goals.
As of v1.11 it does two things: squares the four inboard bushing axes
to the design yaw, AND <b>pulls the kingpin into the 2D sketch
plane</b> so a true one-sketch corner can be drawn — removing the
knuckle twist you'd otherwise see in side view. Note the geometry: a
kingpin lying in a sketch that's tilted by the kickup angle ends up
with caster roughly EQUAL to the kickup. <b>By default MICKSUS keeps
the kingpin planar</b> (Setup shows caster following the kickup), so
seeds and Re-square both build a twist-free corner. If you need caster
independent of kickup, tick <b>"Independent caster (allow kingpin
twist)"</b> in Setup — then your desired-caster value is used, the
kingpin sits off the sketch by |caster − kickup|, and Re-square leaves
it alone. The status bar reports any caster shift; Undo-able.</p>

<h2>Frame backdrop auto-alignment (v1.6)</h2>
<p>Importing a frame mesh now <b>auto-aligns it to the chassis
convention</b> (scale guessed from size, rotated onto the X-forward
kinematics) — with the default seed offsets, the design should already
sit ON the frame with nothing to adjust. The transform controls remain
for odd meshes, but a warning appears if you move the frame off the
canonical alignment: the backdrop marks where the REAL chassis is, so if
the suspension doesn't meet it, move the <i>suspension</i>, not the
frame — otherwise exported coordinates won't land on the real car.</p>

<h2>Halfshafts at seed time (v1.6)</h2>
<p>Tick <b>"Driven axle: model halfshaft"</b> in the Setup panel before
generating — being a driven axle is a decision you make before drawing
geometry. The seed auto-places the inner CV joint so the shaft runs
level at <i>mid-travel</i> (the placement that minimizes plunge over a
bump-biased range) and the violet shaft line shows in the 3D view on
both sides. Fine-tune the joint position and datasheet limits in the
Halfshafts panel afterwards.</p>

<h2>Move both axles (v1.6)</h2>
<p>The <b>"Both axles"</b> checkbox next to the travel slider heaves the
front and rear together (each clamped to its own range) so you can watch
the whole car compress instead of one end at a time.</p>

<h2>The design workflow (v1.5)</h2>
<p><b>Seed → place mounts → checklist green → Optimize → snapshot →
export/sync.</b> The <b>Checklist</b> dock grades the design live:
articulation vs the travel you asked for, one-sketch arms, bump steer,
roll-centre band, CV limits — hover any ✗ for the number behind it.
Edits that shrink the reachable travel are no longer rejected: the swept
range <b>auto-clamps</b> and the checklist shows the shortfall. A rolling
<b>autosave</b> runs a few seconds after every change (File → Restore
autosave).</p>

<h2>Drag hardpoints in the 3D view (v1.5)</h2>
<p>Click any white hardpoint sphere of the axle you're editing and
<b>drag it</b>. Motion follows the view plane; <b>hold X, Y or Z</b> to
lock the drag to that axis of the active coordinate convention (so Y is
fore-aft in chassis coords). The linkage re-solves live while you drag;
releasing runs the full pipeline (sweep, checklist, undo — Ctrl+Z takes
the whole drag back). <b>Esc</b> cancels mid-drag. Dragging jumps to
ride height first: you are editing the design positions. The optional
<b>Snap to frame</b> toggle (off by default — real mounts sit on welded
tabs offset from the tubes) pulls a released point onto the nearest
frame-mesh surface within ~1.2 in.</p>

<h2>Design candidates (v1.5)</h2>
<p>The <b>Candidates</b> dock stores named whole-vehicle snapshots
(front + rear + vehicle params + headline metrics, frozen at snapshot
time). Double-click a name to rename, compare the metric columns side
by side, and <b>Load</b> to hop back to any of them. Candidates save
inside the .MICK. Snapshot before you hop — loading doesn't auto-save
your current state.</p>

<h2>Onshape CAD build</h2>
<p><b>File → Build/update CAD in Onshape</b> is the single CAD handoff:
it creates or updates — in place — a Variable Studio of hardpoints, a
Feature Studio + Part Studio that draw the points, sketch planes and
halfshaft lines, and a design-notes PDF. Push again and it moves the
existing geometry rather than duplicating it, so downstream references
survive. <b>Open design from Onshape document</b> pulls a stored design
back. (The older "Push hardpoints" and "Sync design" items were folded
into this one build step.)</p>

<h2>Chassis coordinate convention (v1.4)</h2>
<p>The <b>Coords</b> dropdown in the toolbar switches how coordinates
READ everywhere (table, CSV export, Onshape push, CG readout): the
tool's native frame (X forward) or the team chassis convention
(fore-aft on Y, lateral on X). The math underneath never changes, and
saved projects remember the setting. If your frame's nose points toward
&minus;Y in Onshape, pick "Chassis (&minus;Y fwd)" instead.</p>

<h2>Halfshafts (v1.4)</h2>
<p>The <b>Halfshafts</b> panel models a CV axle per axle end: tick the
axle, place the inner CV joint (gearbox output flange), and set the
joint's rated max angle and plunge from its datasheet. The tool then
tracks <b>CV angle</b> (worst of inner/outer joints) and <b>plunge</b>
through the whole travel sweep — two new graph channels — and warns
when a limit is exceeded. Warnings, not hard stops: move the inner
mount (hint: a shaft that runs level at <i>mid-travel</i> rather than
at ride height plunges far less) or accept it knowingly. Works for all
suspension types (the outer joint is taken at the wheel centre).</p>

<h2>Onshape-aligned coordinates (v1.3)</h2>
<p>New seeds are shifted <b>+39 in forward and −14 in down</b> (editable
in Setup: "Seed offset X/Z") so the numbers in the table paste straight
into the team's Onshape frame. Consequence: the ground is NOT at z = 0 —
it sits at −ride-height (the tool draws it there, and all
ground-referenced metrics use the tire contact plane, so nothing else
changes). Set both offsets to 0 for classic ground-at-zero coordinates.</p>

<h2>One-sketch manufacturability (v1.3)</h2>
<p>The team's arms are designed on ONE 2D sketch with the bushing tubes
normal to it. That works if and only if the upper and lower bushing axes
stay <b>parallel</b>. The "Sketch fit" line under the hardpoint table
watches this live (green = OK, red = the two arms now demand different
sketch planes), and <b>running Optimize automatically re-squares the
axes</b> (smallest possible correction: each pair keeps its midpoint and
spread). Ball joints sitting fore/aft of the sketch are fine — that
offset is your caster, not an error.</p>

<h2>The three suspension types</h2>
<p>The <b>Suspension type</b> dropdown re-seeds the axle you're editing
as the chosen type. Front and rear axles can be different types.</p>

<h3>Double wishbone</h3>
<p>Two A-arms + tie rod. The classic Baja front end. All panels support
it fully. The inboard bushing pairs stay normal to the 2D design sketch
(the kickup convention) — the optimizer preserves this automatically.</p>

<h3>Trailing arm</h3>
<p>One rigid arm swinging on a chassis pivot axis (two bushings), wheel
rigidly attached. With the axis purely lateral, camber and toe never
change ("pure" trailing arm). <b>Skew the axis in plan view</b> (tweaks
panel: "pivot plan skew") and you get a semi-trailing arm — camber and
toe now change through travel. Caster/KPI read blank: there is no
steering knuckle.</p>

<h3>Multilink (5-link) — read this before using it</h3>
<p>Five independent rods locate the upright — think of it as a double
wishbone taken apart: each A-arm becomes <b>two separate links</b>, plus
the tie rod. That's exactly how the seed starts: as your double-wishbone
equivalent. The naming in the table:</p>
<ul>
<li><b>Upper link A / B</b> (links 1–2): together they do the upper
A-arm's job. Where their extended lines cross = the <i>virtual upper
ball joint</i>.</li>
<li><b>Lower link A / B</b> (links 3–4): the lower A-arm's job; their
crossing = the <i>virtual lower ball joint</i>.</li>
<li><b>Toe link</b> (link 5): the tie rod. Its chassis end rides the
steering rack on a front axle.</li>
</ul>
<p><b>Why bother?</b> Separating the links lets the virtual ball joints
<i>move through travel</i> — you can get camber/toe behaviour no rigid
A-arm allows. The tool computes the <b>virtual kingpin</b> (the line
through the two virtual ball joints) so caster/KPI/scrub still read out.
<b>How to use it:</b> seed as multilink, then move ONE link end at a
time in the table and watch the curves. Small moves — the geometry is
sensitive. Undo (Ctrl+Z) is your friend.</p>

<h2>3D view</h2>
<ul>
<li><b>Fit view</b> reframes everything; Front/Side/Top/Iso snap to
orthographic views (parallel projection keeps parallel lines parallel —
trust the Side view when checking the bushing axes).</li>
<li><b>Markers</b>: orange = front-view instant centre (with its
construction line from the contact patch), cyan = side-view IC, magenta
= roll centres and the roll axis joining them.</li>
<li><b>Heave / Roll</b> (next to Play): Heave moves the wheels together;
<b>Roll</b> moves left up while right goes down — with "Mirror right
side" on, you see the whole car roll, the RC migrate off-centreline, and
the roll axis move. Play animates either mode.</li>
<li><b>Steer slider</b>: turns the front rack within your set limit.
With the mirror on, BOTH wheels steer the same physical direction (the
right side gets its own solve at the opposite rack sign), so what you
see is the real car turning.</li>
<li><b>Camera</b> stays put when you edit values — only Fit view and
the view buttons move it. Orbiting always pivots about the model (the
rotation centre is re-anchored on every geometry change).</li>
<li><b>CG</b> checkbox shows the yellow CG sphere.</li>
</ul>

<h2>Graphs &amp; analysis</h2>
<ul>
<li><b>Graphs ▾</b> picks which curves show. <b>Anti-squat (drive) %</b>
uses the wheel-centre support line (inboard drive, the Baja rear);
<b>anti-dive/lift (brake) %</b> uses the contact-patch line scaled by
your brake split. Both need the Vehicle panel's CG filled in.</li>
<li><b>Sweeps ▾</b>: roll sweep (ground camber/toe, RC migration), pitch
sweep (needs both axles), steering sweep (Ackermann %, turn diameter),
bump-steer map (toe over travel × rack).</li>
<li><b>Compare ▾</b>: pin snapshots of the current curves, tweak, and
compare against them.</li>
</ul>

<h2>Getting your design out</h2>
<ul>
<li><b>File → Save</b>: everything in one .MICK project file.</li>
<li><b>File → Export/Copy hardpoints CSV</b>: for Excel or the Onshape
paste feature (see onshape/README.md in the repo).</li>
<li><b>File → Build/update CAD in Onshape</b>: builds points, sketch
planes and a notes PDF via the API — re-running updates them in place.</li>
<li><b>File → Export report CSV</b>: the design table + all curves.</li>
</ul>

<h2>Tips</h2>
<ul>
<li><b>Undo</b> (Ctrl+Z or the toolbar button) steps back up to 15
geometry changes per axle.</li>
<li>Scroll wheel only edits a field after you CLICK into it — no more
drive-by edits.</li>
<li>If the viewport ever shows just floating dots, the last edit made an
impossible linkage and got rejected — check the status bar, then Undo if
needed.</li>
<li>Sign conventions: camber negative = top leans inboard; toe positive
= toe-in; caster positive = kingpin top rearward; motion ratio =
wheel travel / shock travel.</li>
</ul>
"""
