Here's my plan:

     1. Remove unused crossterm from Cargo.toml
     2. Fix rate limiter bug (line 445-448)
     3. Fix unsafe array indexing in translate_message (lines
        296-297)
     4. Add PWM driver Drop for failsafe shutdown
     5. Simplify config by replacing TryFrom + expect chain with
        direct validation
     6. Replace linear channel lookup in RC_CHANNELS_OVERRIDE
        with O(1) HashMap lookup
