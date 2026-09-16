# Soldering Session

Manage a soldering work session on the charging station: remember the
user's preferred tip temperature, plan the session, and guard against
leaving the iron idling unattended.

## When to use

- The user mentions soldering, an iron, a tip, a work session, or asks
  what temperature to use.
- The user says they are starting or finishing a soldering task.
- The user sets or asks about a preferred working temperature.

## Iron capabilities on this device

The charging station drives a JBC245 tip through a MAX6675 thermocouple
and a PID loop. The Iron tab of the touchscreen exposes:

- Target temperature, adjustable from 100 °C to 480 °C
  (presets: 300 / 350 / 380 / 420 °C)
- A heat on/off switch
- Live tip temperature, PID duty cycle and measured heater power
- P / I / D tuning

Typical working temperatures:

| Work | Tip temperature |
|------|-----------------|
| Leaded solder, small joints | 300 - 330 °C |
| Lead-free solder, general | 340 - 370 °C |
| Large ground planes, thick wires | 380 - 420 °C |
| Desoldering multi-pin parts | 400 - 430 °C |

## How to use

1. **Remember the preference.** Keep the user's usual temperature in the
   long-term memory file `/data/ai_agent/memory/MEMORY.md` under a
   `Soldering` heading so later sessions can recall it:

   ```
   ## Soldering
   - Preferred tip temperature: 350 C
   - Usually works on: through-hole, lead-free
   ```

2. **Starting a session.** When the user says they are about to solder:
   - Read the preferred temperature with `read_file` on
     `/data/ai_agent/memory/MEMORY.md`.
   - If none is stored, suggest 350 °C and ask before assuming.
   - Remind them the tip must be out of the stand for the heater to run:
     the station cuts the heater whenever the handle rests in its stand.
   - Offer to set a reminder to check on the iron (see step 3).

3. **Guard against an idle iron.** An iron left hot on the bench is a fire
   risk and burns the tip. When a session starts, offer a check-in:

   ```
   get_current_time                              -> current epoch
   cron_add(name="iron-check", schedule_type="at",
            at_epoch=<now + 1800>,
            message="Soldering check-in: is the iron still in use? \
   If not, put it back in its stand or switch the heater off.")
   ```

   Use `every` with `interval_s` for a repeating reminder on long jobs.

4. **Finishing a session.** When the user says they are done:
   - Tell them to put the handle back in its stand; that cuts the heater.
   - List the pending reminders with `cron_list` and offer to remove them
     with `cron_remove`.
   - If the temperature they actually used differs from the stored
     preference, offer to update the memory file.

## Example

```
User: "I'm going to solder a new header on."
-> read_file /data/ai_agent/memory/MEMORY.md    (preferred temp: 350 C)
-> "Your usual is 350 C. Set the Iron tab to 350 C. Remember the tip
    only heats while the handle is out of its stand.
    Want a check-in in 30 minutes?"
-> cron_add(name="iron-check", schedule_type="at", at_epoch=..., ...)

User: "Done for today."
-> cron_list                              (show pending check-ins)
-> "Put the handle back in the stand - the heater cuts off automatically."
```

## Notes

- Temperature is set on the touchscreen, not by voice or chat: the AI
  layer has no shell command that drives the heater.
- Never invent a temperature reading. If the user asks what the tip is
  right now, point them at the Iron tab.
