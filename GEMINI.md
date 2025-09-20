# GEMINI.md - Arbeitsanweisungen für den KI-Assistenten

Dieses Dokument beschreibt die Kernprinzipien und projektspezifischen Regeln, nach denen der KI-Assistent arbeitet.

## 1. Allgemeine Prinzipien

-   **Kontext-Treue:** Ich halte mich strikt an die Konventionen des Projekts (Code-Stil, Bibliotheken, Architektur). Vor Änderungen analysiere ich den bestehenden Code.
-   **Sicherheit:** Kritische Befehle, die das Dateisystem oder den Systemzustand verändern, werden vor der Ausführung erklärt. Ich verarbeite oder speichere niemals sensible Daten wie Passwörter oder Tokens.
-   **Effizienz und Präzision:** Meine Antworten sind kurz und direkt. Ich nutze Werkzeuge für Aktionen und Text nur für die Kommunikation.

## 2. Technische Arbeitsweise

-   **Dateisystem:** Ich verwende für alle Dateioperationen ausschließlich absolute Pfade.
-   **Shell-Befehle:** Modifizierende Befehle werden erklärt. Langlaufende Prozesse starte ich als Hintergrund-Jobs (`&`).
-   **Verifizierung:** Nach Code-Änderungen stelle ich durch Kompilieren oder das Ausführen von Tests (sofern im Projekt definiert) sicher, dass die Änderungen korrekt sind.

## 3. Git-Workflow

-   **Analyse:** Vor einem Commit analysiere ich den Zustand des Repositories mit `git status`, `git diff` und `git log`, um Konsistenz zu gewährleisten.
-   **Commit-Erstellung:** Ich schlage aussagekräftige Commit-Nachrichten vor, die die Änderungen und deren Zweck zusammenfassen.
-   **Manuelle Bestätigung:** Ich führe niemals `git push` aus, ohne explizit dazu aufgefordert zu werden.

## 4. Projektspezifische Konventionen (DShotRMT)

-   **Technologie-Stack:** C++, `arduino-cli`
-   **Build-Befehl:** `arduino-cli compile --fqbn esp32:esp32:esp32 <sketch-pfad>`
-   **Namenskonventionen:** `_private_var` für private Member, `camelCase` für öffentliche Funktionen.
-   **Sprache im Code:** Kommentare, Variablen und Funktionsnamen sind ausschließlich auf Englisch.
-   **Interaktion:** Code-Vorschläge und Änderungen werden begründet, um deren Vorteile aufzuzeigen.
