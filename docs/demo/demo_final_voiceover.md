# SafeRoom — Voice-over script for `demo final.mp4`

Narration for the recorded demo (`2026-08-01/demo final.mp4`). English, first person,
read against picture after the edit is locked.

- **Pace:** ~145 words per minute. Calm, unhurried; this is a product walkthrough, not
  a lecture.
- **Silence is part of the script.** Say nothing for a beat at each fall, and leave the
  immobility wait mostly unnarrated — the dashboard flipping on its own sells it better
  than words.
- **No statistics here.** Dataset size, cross-validation and false-alarm rates belong to
  the slide presentation, not to this video.

## Running order

| # | Scene | Target | Cumulative |
|---|---|---|---|
| 1 | Opening — what this demo is | ~30 s | 0:30 |
| 2 | The room and where the sensor goes | ~45 s | 1:15 |
| 3 | The dashboard | ~60 s | 2:15 |
| 4 | Presence, and how alerts reach the phone | ~40 s | 2:55 |
| 5 | Room view, and a normal day | ~80 s | 4:15 |
| 6 | Harder everyday movements | ~45 s | 5:00 |
| 7 | Falls | ~90 s | 6:30 |
| 8 | Staying down — the immobility alert | ~65 s | 7:35 |
| 9 | Close | ~45 s | 8:20 |

Roughly eight minutes of speech across the whole cut. Trim the optional sentences
(marked *optional*) if the picture runs shorter.

---

### Scene 1 — Opening: what this demo is (~30 s)

**[ON SCREEN]** Title or opening shot of the room.

**[VO]**
> This is the SafeRoom demo. In the next few minutes I am going to show you how the
> system behaves in everyday use — not in a lab, but in a real home, in real time.
> You will see three kinds of situation: ordinary daily activity, simulated falls, and
> a simulated faint. For each one you will see how SafeRoom reacts: where the sensor
> has to be installed, what the caregiver sees on the dashboard, and how the alerts
> arrive on a phone.

---

### Scene 2 — The room and where the sensor goes (~45 s)

**[ON SCREEN]** The real living room. Pan across the space, then a close-up of the
SafeRoom unit on the wall, and the Raspberry Pi next to it.

**[VO]**
> This is the room: a normal living room, about five metres by three point seven. And
> this is the whole system — one sixty-gigahertz millimetre-wave radar and a Raspberry
> Pi that does all the processing. There is no video camera watching this room.
>
> Placement matters, and it is simple. The unit goes high on a wall, about two metres
> up, tilted slightly downwards, facing into the room so that the floor area is covered.
> I have it on the short wall, roughly one metre from the corner, so the beam spreads
> across the whole living area. Anywhere with a clear view of the space works — what you
> want to avoid is putting it behind furniture or aiming it into a wall.

---

### Scene 3 — The dashboard (~60 s)

**[ON SCREEN]** Screen recording of the dashboard. Cursor moves over each element as
it is described: status banner, Recent activity, connection indicators, Room view button.

**[VO]**
> This is the dashboard, and it is what the caregiver actually uses. Anyone looking after
> someone at home opens this page and knows, at a glance, how that person is.
>
> The big panel at the top is the status. Right now it is calm; when something happens it
> turns red and names the alert. Underneath it, Recent activity: a timestamped history of
> everything the system has registered — someone entering the room, leaving it, a fall,
> a period of immobility. Nothing is lost; if the caregiver was away from the screen, the
> history tells them what happened and when.
>
> On the side there are two connection indicators, one for the radar and one for the
> thermal camera, so you can see at any moment that the sensors are alive and streaming.
> And there is a Room view button — I will open that one in a moment, once there is
> somebody in the room to look at.

---

### Scene 4 — Presence, and how alerts reach the phone (~40 s)

**[ON SCREEN]** Split or side-by-side: dashboard on one side, phone screen recording on
the other. Empty room, then someone walks in.

**[VO]**
> Before anything else, let me show you the messaging side, because every alert you see
> from here on also lands on a phone.
>
> With nobody in the room, the dashboard says the room is empty, and a message goes out
> saying exactly that. The moment I walk in, the status flips to "all clear, presence
> detected" — and a second message arrives confirming there is someone in the room.
>
> This scene is only to show you how the messages look and how quickly they arrive. From
> here on, you will see them in real situations.

---

### Scene 5 — Room view, and a normal day (~80 s)

**[ON SCREEN]** Room view opens: 2D floor plan with the green dot, thermal panel below.
Then the everyday-activity run — walking, sitting on the sofa, watching TV, opening
drawers — with the dot following.

**[VO]**
> This is Room view. On top, a two-dimensional plan of the room with my position on it.
> Below, a thermal view of part of the room.
>
> One word about that thermal image, because it is important and it is easy to
> misunderstand. It is a very low-resolution infrared array — with a better camera it
> would look much sharper. But its resolution does not matter here, because it is not
> part of the detection. The infrared camera does not influence this project at all: it
> takes no decisions, it triggers no alerts, it is not an input to anything. It is on
> screen for one reason only — so you can see that what the radar sees and the video of
> me moving around the room were recorded at the same moment. Every detection you are
> about to see comes from the radar alone.
>
> Now, an ordinary day. I walk around the room, I sit down, I watch television, I open a
> drawer, I get up again. Watch the green dot: while I move, it follows me around the
> plan, picking up my position accurately the whole time. And at no point does the system
> raise a fall.

---

### Scene 6 — Harder everyday movements (~45 s)

**[ON SCREEN]** Lying down far from the sensor, crouching in several different ways,
sitting down on the floor. Dashboard stays green throughout.

**[VO]**
> Now the difficult part — normal life, but the awkward cases. I lie down a long way from
> the sensor, at the edge of its range. I crouch down in several different ways. I even
> sit down on the floor, which is something an older person is very unlikely to do unless
> they have fallen.
>
> These are exactly the movements that fool a naive detector, because every one of them
> ends with a body suddenly close to the floor. And throughout all of it, two things
> hold: the radar keeps track of me at every moment, and no fall alert is raised.

---

### Scene 7 — Falls (~90 s)

**[ON SCREEN]** Beat 1 — setting up the mattress, then a committed forward fall. Cut the
dashboard to full frame at the moment of impact: red banner, red dot, history rows, the
Acknowledge button. Beat 2 — backward trip fall, with the phone screen capture. Closing
still — phone message timestamps next to the Recent activity list.

**[VO]** *(let the first fall land in silence, then speak)*
> Now let me set up the situation the system exists for.
>
> First, a forward fall. The moment I go down, the sensor picks it up: the dashboard turns
> red, and my position on the floor plan turns red with it. If you look at the history,
> you can see two separate entries for the same event. "Fall detected" comes from the
> rule-based detector — the hand-tuned logic that watches how fast I dropped. "Fall
> confirmed" comes from the learned detector, the machine-learning model that scores the
> whole pattern of the movement. Two independent detectors, both agreeing that this was a
> fall.
>
> There is also an Acknowledge button. Once the caregiver has attended the person, they
> press it, the alert clears, and the system carries on watching. Nothing has to be
> restarted.
>
> Now a more controlled fall, and a more typical one for an older person: tripping and
> going over backwards. The behaviour is the same — red on the dashboard, red on the floor
> plan, both detectors firing. And here is the phone, recorded at the same time, receiving
> the fall notification.
>
> This last image puts the two side by side: the timestamps of the messages on my phone,
> and the Recent activity list on the dashboard. Same events, same order, same times.

---

### Scene 8 — Staying down: the immobility alert (~65 s)

**[ON SCREEN]** Forward fall, then staying motionless on the floor. Dashboard shows the
fall alert first; after the wait it flips to IMMOBILITY ALERT and a "Possible faint" row
appears. Final still: the phone conversation with both messages.

**[VO]** *(the wait is mostly silence — let the dashboard flip on its own)*
> One more situation, and it is the dangerous one. I trip forward, I fall, and this time
> I stay on the ground — as if I had fainted, or simply could not get back up.
>
> The first thing that happens is what you have already seen: the fall is detected and
> the alert goes out. But then nothing changes. I am at floor level and I am not moving.
> After about thirty seconds of that — long enough to rule out someone who bent down to
> pick something up — the system escalates. The alert changes to an immobility alert, and
> a second, different message goes to the phone.
>
> That distinction matters. A fall someone gets up from is one thing. A person on the
> floor who has stopped moving is an emergency, and the longer they lie there the worse
> the outcome. Here are the messages as they arrived.

---

### Scene 9 — Close (~45 s)

**[ON SCREEN]** Final dashboard shot with the session's full history, or a wide shot of
the room.

**[VO]**
> And that concludes the demo. What you have seen here are just a few examples — during
> the project I recorded and simulated many more kinds of fall, from many positions and
> directions in the room, and it is that work that makes the system behave as reliably as
> it does here.
>
> One radar sensor, one Raspberry Pi. Everything processed inside the home, with no video
> camera, no cloud service and nothing for the person to wear or remember to press. It
> detects presence, it detects falls, it detects when someone stays down — and it puts
> that on a caregiver's phone within seconds. That is SafeRoom. Thank you for watching.

---

## Wording that must match the screen

Keep these exact, since the viewer reads them at the same time:

| Narration says | Screen / phone shows |
|---|---|
| "the room is empty" | `Room empty — no one being tracked` · `⚪ Room empty — no presence at HH:MM:SS` |
| "all clear, presence detected" | `All clear` / `Presence detected` · `🟢 Presence detected at HH:MM:SS` |
| "Fall detected" (rule-based) | banner `FALL DETECTED`, history row `Fall detected` |
| "Fall confirmed" (learned) | banner `FALL CONFIRMED`, history row `Fall confirmed` |
| "immobility alert" | banner `IMMOBILITY ALERT`, history row `Possible faint` |
| fall notification | `🚨 FALL detected at HH:MM:SS` |
| faint notification | `🆘 FAINT detected at HH:MM:SS` |

Record with the dashboard in **EN** (the `EN` button in the header). The Spanish strings
still carry an "(IR)" suffix on *Caída confirmada*, which would contradict the scene 5
narration.
