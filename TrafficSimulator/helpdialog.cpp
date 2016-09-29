#include "helpdialog.h"
#include "ui_helpdialog.h"

#include "constants.h"

HelpDialog::HelpDialog(QWidget *parent) :
		QDialog(parent),
		ui(new Ui::HelpDialog) {
	ui->setupUi(this);
	ui->helpContentTextBrowser->setText(QString::fromStdString(Help::MAIN_HELP_MESSAGE));
}

HelpDialog::~HelpDialog() {
	delete ui;
}

void HelpDialog::on_closePushButton_clicked() {
	setVisible(false);
}
