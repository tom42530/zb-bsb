#include "bsb_freshness.h"

namespace bsb {

MeasurementWatch::Entry* MeasurementWatch::find(uint32_t field_id) {
    for (size_t i = 0; i < count_; ++i) {
        if (entries_[i].field_id == field_id) {
            return &entries_[i];
        }
    }
    return nullptr;
}

const MeasurementWatch::Entry* MeasurementWatch::find(uint32_t field_id) const {
    for (size_t i = 0; i < count_; ++i) {
        if (entries_[i].field_id == field_id) {
            return &entries_[i];
        }
    }
    return nullptr;
}

bool MeasurementWatch::track(uint32_t field_id, uint64_t t_now) {
    if (find(field_id)) {
        return true;
    }
    if (count_ >= MAX_FIELDS) {
        return false;
    }
    Entry& e        = entries_[count_++];
    e.field_id      = field_id;
    e.last_value_ms = t_now;
    e.unavailable   = false;
    return true;
}

bool MeasurementWatch::note_value(uint32_t field_id, uint64_t t_now) {
    Entry* e = find(field_id);
    if (!e) {
        return false;
    }
    const bool was_unavailable = e->unavailable;
    e->last_value_ms = t_now;
    e->unavailable   = false;
    return was_unavailable;
}

bool MeasurementWatch::note_no_value(uint32_t field_id) {
    Entry* e = find(field_id);
    if (!e || e->unavailable) {
        return false;
    }
    e->unavailable = true;
    return true;
}

size_t MeasurementWatch::expire(uint64_t t_now, uint64_t max_age_ms, uint32_t* out_fields, size_t out_capacity) {
    size_t written = 0;
    if (!out_fields) {
        return 0;
    }
    for (size_t i = 0; i < count_ && written < out_capacity; ++i) {
        Entry& e = entries_[i];
        if (e.unavailable) {
            continue;
        }
        // Une horloge qui reculerait ne doit jamais faire expirer une mesure
        const uint64_t age_ms = (t_now >= e.last_value_ms) ? (t_now - e.last_value_ms) : 0;
        if (age_ms >= max_age_ms) {
            e.unavailable         = true;
            out_fields[written++] = e.field_id;
        }
    }
    return written;
}

bool MeasurementWatch::is_unavailable(uint32_t field_id) const {
    const Entry* e = find(field_id);
    return e && e->unavailable;
}

size_t MeasurementWatch::unavailable_count() const {
    size_t n = 0;
    for (size_t i = 0; i < count_; ++i) {
        if (entries_[i].unavailable) {
            ++n;
        }
    }
    return n;
}

uint32_t MeasurementWatch::unavailable_field(size_t index) const {
    for (size_t i = 0; i < count_; ++i) {
        if (entries_[i].unavailable) {
            if (index == 0) {
                return entries_[i].field_id;
            }
            --index;
        }
    }
    return 0;
}

} // namespace bsb
