using System;
using UnityEngine;
using Preliy.Flange;

namespace Preliy.Flange.Orchestration
{
    public enum ManipulationTool
    {
        None = 0,
        Syringe = 1,
        Screwdriver = 2
    }

    /// <summary>
    /// Desiderata dell'utente. Il builder li traduce in stati atomici.
    /// </summary>
    [Serializable]
    public struct ManipulationTaskRequest
    {
        [Tooltip("Tool da montare prima di iniziare. None = nessun tool.")]
        public ManipulationTool Tool;

        [Tooltip("Colore di ogni vite da trattare, nell'ordine desiderato.")]
        public ScrewColor[] ScrewColors;

        [Tooltip("Se true, il task usa anche il tipo di colla identificato da GlueColor.")]
        public bool UseGlue;

        public GlueColor GlueColor;

        [Range(0f, 1f)]
        [Tooltip("Pressione normalizzata 0..1. Valori anomali diventano 1; la durata è pressione × 8 secondi.")]
        public float Pressure;

        public SolutionIgnoreMask IgnoreMask;
    }
}
