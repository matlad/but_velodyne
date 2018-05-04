/**
 * @file     color.h
 * @encoding UTF-8
 * @date     13.11.2015
 * @author   Adam Matl <xmatla00@stud.fit.vutbr.cz>
 * @brief Escape sekvence pro barvy v terminálu
 * Vytvořeno v rámci knihovny matla
 */

#ifndef COLOR_H
#define COLOR_H

/// Seznam barev textu ////////////////////////////////////////////////////////
#define COLOR_DEFAULT "\033[0m"
#define COLOR_BLACK "\033[30m"
#define COLOR_RED "\033[31m"
#define COLOR_GREEN "\033[32m"
#define COLOR_YELLOW "\033[33m"
#define COLOR_BLUE "\033[34m"
#define COLOR_MAGENTA "\033[35m"
#define COLOR_CYAN "\033[36m"
#define COLOR_LIGHT_GRAY "\033[37m"
#define COLOR_DARK_GRAY "\033[90m"
#define COLOR_LIGHT_RED "\033[91m"
#define COLOR_LIGHT_GREEN "\033[92m"
#define COLOR_LIGHT_YELLOW "\033[93m"
#define COLOR_LIGHT_BLUE "\033[94m"
#define COLOR_LIGHT_MAGENTA "\033[95m"
#define COLOR_LIGHT_CYAN "\033[96m"
#define COLOR_WHITE "\033[97m"

/// Barvy pro typy výstupů /////////////////////////////////////////////////////
#define COLOR_DEBUG COLOR_YELLOW
#define COLOR_DEBUG_END COLOR_DEFAULT


////////////////////////////////////////////////////////////////////////////////
#endif /* COLOR_H */

