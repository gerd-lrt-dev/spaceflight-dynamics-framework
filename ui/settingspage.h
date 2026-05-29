/**
 * @file settingspage.h
 * @brief Placeholder settings page for future SDF releases.
 */

#ifndef SETTINGSPAGE_H
#define SETTINGSPAGE_H

#include <QWidget>

/**
 * @class SettingsPage
 * @brief Placeholder widget for future application settings.
 *
 * This page intentionally acts as a roadmap placeholder
 * for the planned v0.2 SDF Research Release.
 */
class SettingsPage : public QWidget
{
    Q_OBJECT

public:

    /**
     * @brief Constructs the settings placeholder page.
     * @param parent Optional parent widget.
     */
    explicit SettingsPage(QWidget *parent = nullptr);

    /**
     * @brief Default destructor.
     */
    ~SettingsPage() = default;

private:

    /**
     * @brief Builds the settings placeholder UI.
     */
    void setupUI();
};

#endif // SETTINGSPAGE_H
