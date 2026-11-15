#pragma once

#include <cstddef>
#include <cstdint>

namespace bsb {

/**
 * @brief Suivi de fraîcheur des mesures lues sur le bus BSB.
 *
 * Une mesure affichée dans Home Assistant ne doit pas rester figée sur sa dernière valeur quand le
 * régulateur ne la fournit plus : câble débranché, régulateur hors tension, sonde en défaut (réponse
 * « --- »). Ce suivi dit, pour chaque paramètre surveillé, s'il est encore alimenté. Il ne publie rien
 * lui-même : c'est l'appelant qui marque alors la mesure indisponible.
 *
 * Logique pure, sans dépendance matérielle : les instants (millisecondes) sont fournis par l'appelant.
 */
class MeasurementWatch {
public:
    static constexpr size_t MAX_FIELDS = 32;

    /**
     * @brief Commence à surveiller un paramètre. L'instant fourni sert de point de départ : un paramètre
     *        jamais lu devient indisponible une fois le délai écoulé depuis cet instant.
     * @return false si la table est pleine (le paramètre n'est alors pas surveillé)
     */
    bool track(uint32_t field_id, uint64_t t_now);

    /**
     * @brief Une valeur exploitable vient d'être lue.
     * @return true si le paramètre était tenu pour indisponible (il redevient disponible)
     */
    bool note_value(uint32_t field_id, uint64_t t_now);

    /**
     * @brief Le régulateur vient de répondre qu'il n'a pas de valeur (« --- », mesure hors plage).
     * @return true si le paramètre vient de passer à indisponible (false s'il l'était déjà, ou s'il
     *         n'est pas surveillé)
     */
    bool note_no_value(uint32_t field_id);

    /**
     * @brief Passe à indisponible les paramètres restés sans valeur depuis au moins `max_age_ms`.
     *
     * Les paramètres qui ne tiennent pas dans `out_fields` ne sont pas marqués : ils seront rendus par
     * l'appel suivant.
     *
     * @param out_fields    Reçoit les paramètres qui viennent de passer à indisponible
     * @param out_capacity  Nombre de places dans `out_fields`
     * @return Nombre de paramètres écrits dans `out_fields`
     */
    size_t expire(uint64_t t_now, uint64_t max_age_ms, uint32_t* out_fields, size_t out_capacity);

    bool   is_unavailable(uint32_t field_id) const;
    size_t unavailable_count() const;
    size_t tracked_count() const { return count_; }

    /** Paramètre indisponible de rang `index` (0 .. unavailable_count() - 1) ; 0 au-delà. */
    uint32_t unavailable_field(size_t index) const;

private:
    struct Entry {
        uint32_t field_id      = 0;
        uint64_t last_value_ms = 0;
        bool     unavailable   = false;
    };

    Entry*       find(uint32_t field_id);
    const Entry* find(uint32_t field_id) const;

    Entry  entries_[MAX_FIELDS] = {};
    size_t count_               = 0;
};

} // namespace bsb
