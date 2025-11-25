import React from "react";

interface ActivityLogProps {
  entries: string[];
  onClose: () => void;
}

const ActivityLog: React.FC<ActivityLogProps> = ({ entries, onClose }) => {
  return <ActivityLogModal entries={entries} onClose={onClose} />;
};

interface ActivityLogModalProps {
  entries: string[];
  onClose: () => void;
}

const ActivityLogModal: React.FC<ActivityLogModalProps> = ({
  entries,
  onClose,
}) => {
  // close on backdrop click, but not when clicking inside the box
  const handleBackdropClick = () => onClose();
  const stopPropagation: React.MouseEventHandler<HTMLDivElement> = (e) =>
    e.stopPropagation();

  return (
    <div className="activity-log-backdrop" onClick={handleBackdropClick}>
      <div className="activity-log-modal" onClick={stopPropagation}>
        <div className="activity-log-modal-header">
          <h2>Activity Log</h2>
          <button
            type="button"
            className="activity-log-close-btn"
            onClick={onClose}
          >
            ✕
          </button>
        </div>

        <div className="activity-log-modal-body">
          {entries.map((entry, i) => (
            <div key={i} className="activity-log-line">
              {entry}
            </div>
          ))}
        </div>
      </div>
    </div>
  );
};

export default ActivityLog;
