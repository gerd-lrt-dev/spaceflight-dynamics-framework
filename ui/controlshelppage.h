/**
 * @file controlshelppage.h
 * @brief Help and control reference page for the simulator frontend.
 *
 * Displays the currently supported keyboard control bindings
 * and simulation interaction commands.
 */

#ifndef CONTROLSHELPPAGE_H
#define CONTROLSHELPPAGE_H

#include <QVBoxLayout>
#include <QWidget>

/**
 * @class ControlsHelpPage
 * @brief Displays the current simulator control bindings.
 *
 * This widget provides a keyboard reference page for the
 * unified control input system (D22).
 */
class ControlsHelpPage : public QWidget
{
    Q_OBJECT

public:

    /**
     * @brief Constructs the controls help page.
     * @param parent Optional parent widget.
     */
    explicit ControlsHelpPage(QWidget *parent = nullptr);

    /**
     * @brief Default destructor.
     */
    ~ControlsHelpPage() = default;

private:

    /**
     * @brief Main page layout.
     */
    QVBoxLayout *mainLayout = nullptr;

    /**
     * @brief Builds all UI elements.
     */
    void setupUI();

    /**
     * @brief Creates a visual control entry row.
     *
     * @param key Keyboard key representation.
     * @param description Action description.
     * @return QWidget containing the formatted row.
     */
    QWidget* createControlRow(const QString& key,
                              const QString& description);

    /**
     * @brief Creates a titled control section.
     *
     * @param title Section title.
     * @return QWidget section container.
     */
    QWidget* createSectionTitle(const QString& title);
};

#endif // CONTROLSHELPPAGE_H
