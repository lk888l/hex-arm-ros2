//! Exclusive session admission and ownership.
use super::*;

impl ArmRuntime {
    pub fn acquire(&self, client_name: String) -> Result<(u32, u32, Option<String>)> {
        let mut data = self.data.write();
        anyhow::ensure!(
            !data.closing && !data.damped_stopping,
            "controller is shutting down"
        );
        let event_client_name = client_name.clone();
        if let Some(holder) = &data.session {
            return Ok((0, holder.id, Some(holder.client_name.clone())));
        }
        let id = data.next_session_id.max(1);
        data.next_session_id = data.next_session_id.wrapping_add(1).max(1);
        data.gravity = self.profile.gravity_vector_base_m_s2;
        data.session = Some(SessionLease { id, client_name });
        self.push_event_locked(
            &mut data,
            pb::EventSeverity::Info,
            "session_granted",
            format!("exclusive session {id} granted to {event_client_name}"),
            &[("session_id", id.to_string())],
        );
        Ok((id, 0, None))
    }

    pub async fn release(&self, session_id: u32) -> Result<()> {
        let wait = crate::trace::Span::new("gate_wait", 0, 0);
        let _gate = self.mode_gate.lock().await;
        drop(wait);
        self.ensure_accepting_requests()?;
        self.require_session(session_id)?;
        self.backend
            .disable_all()
            .await
            .context("disable while releasing session")?;
        let mut data = self.data.write();
        data.safety.disable_preserving_fault();
        data.session = None;
        data.command = None;
        data.gravity_comp = None;
        data.gravity = self.profile.gravity_vector_base_m_s2;
        data.disable_pending = false;
        data.next_disable_retry_at = None;
        self.push_event_locked(
            &mut data,
            pb::EventSeverity::Info,
            "session_released",
            format!("exclusive session {session_id} released; arm disabled"),
            &[("session_id", session_id.to_string())],
        );
        Ok(())
    }

    pub(super) fn require_session(&self, session_id: u32) -> Result<()> {
        let data = self.data.read();
        anyhow::ensure!(
            session_id != 0
                && data
                    .session
                    .as_ref()
                    .is_some_and(|session| session.id == session_id),
            "request does not hold the exclusive session"
        );
        Ok(())
    }

    pub(super) fn ensure_accepting_requests(&self) -> Result<()> {
        let data = self.data.read();
        anyhow::ensure!(
            !data.closing && !data.damped_stopping,
            "controller is shutting down"
        );
        Ok(())
    }
}
