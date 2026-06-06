const fs = require('fs');
const path = require('path');

function walk(dir) {
    let results = [];
    const list = fs.readdirSync(dir);
    list.forEach(function(file) {
        file = path.join(dir, file);
        const stat = fs.statSync(file);
        if (stat && stat.isDirectory()) { 
            results = results.concat(walk(file));
        } else if (file.endsWith('.vue')) {
            results.push(file);
        }
    });
    return results;
}

const vueFiles = walk('./src');
let changedFiles = 0;

vueFiles.forEach(file => {
    let content = fs.readFileSync(file, 'utf8');
    let original = content;

    // We'll use a regex that finds class="..." and inside it, removes text-white etc.
    // BUT only if bg-* is not present.
    // Since class attributes can span multiple lines, this is tricky.
    // A simpler approach: regex that matches class attributes.
    const classAttrRegex = /class=(["'])([\s\S]*?)\1/g;

    content = content.replace(classAttrRegex, (match, quote, classList) => {
        // Does this class list contain bg-something?
        // Note: some custom backgrounds like bg-slate-800 are dark, so we keep text-white.
        // If it's bg-transparent or bg-surface, we might want to remove text-white.
        if (/(?:^|\s)bg-(?!transparent|surface|page)[\w-]+/.test(classList)) {
            return match; // Keep it, it has a background
        }
        
        // No explicit background (or it's transparent), so we can safely remove text-white and text-grey-*
        const newClassList = classList.replace(/\btext-(white|grey-[2-6]|slate-[2-5]00)\b/g, '').replace(/\s{2,}/g, ' ').trim();
        return `class=${quote}${newClassList}${quote}`;
    });

    if (content !== original) {
        fs.writeFileSync(file, content);
        changedFiles++;
    }
});

console.log(`Changed ${changedFiles} files.`);
